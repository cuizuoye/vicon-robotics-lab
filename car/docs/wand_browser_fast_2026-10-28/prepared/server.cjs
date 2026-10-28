'use strict';
// Loopback only; independent control/view sockets, one owner, NO physical entry.
const http=require('http'),fs=require('fs'),path=require('path'),crypto=require('crypto');
const {spawn}=require('child_process'),readline=require('readline');
const {WebSocketServer,WebSocket}=require('/usr/share/nodejs/ws');
const {ViewSlot}=require('./view_slot.cjs');
const root=__dirname,args=process.argv.slice(2);let mode='readonly',port=8765,seconds=60;
for(let i=0;i<args.length;i++){
  if(args[i]==='--readonly')mode='readonly';
  else if(args[i]==='--software')throw Error('Private ROS lab has NOT qualified; this release is READONLY ONLY');
  else if(args[i]==='--port')port=Number(args[++i]);
  else if(args[i]==='--seconds')seconds=Number(args[++i]);
  else throw Error('Unknown/forbidden option '+args[i]+'; NO hardware entry');
}
if(!Number.isInteger(port)||port<0||port>65535||!Number.isFinite(seconds)||seconds<1||seconds>120)throw Error('Bad port/duration');
const sessions=path.join(root,'sessions');fs.mkdirSync(sessions,{recursive:true,mode:0o700});
const session=fs.mkdtempSync(path.join(sessions,'fast_'+mode+'_'));fs.chmodSync(session,0o700);
const token=crypto.randomBytes(24).toString('hex'),steady=()=>Number(process.hrtime.bigint())/1e9;
let actualPort=port,owner=null,child=null,ending=false,lastState=null,lastLoop=steady(),maxLoop=0;
const metrics={control_bytes:0,view_bytes:0,max_control_buffer:0,max_input_buffer:0,control_messages:0};
setInterval(()=>{
  const n=steady();maxLoop=Math.max(maxLoop,n-lastLoop);lastLoop=n;
  if(owner?.opened&&!owner.revoked&&owner.viewSentAt!==null&&n-owner.viewSentAt>.100)revoke('VIEW_ACK_TIMEOUT_100MS');
},10).unref();
const hostOK=h=>h===`127.0.0.1:${actualPort}`||h===`localhost:${actualPort}`;
const originOK=h=>h===`http://127.0.0.1:${actualPort}`||h===`http://localhost:${actualPort}`;
const files={'/':['index.html','text/html'],'/app.js':['app.js','text/javascript'],
  '/input_guard.js':['input_guard.js','text/javascript'],'/style.css':['style.css','text/css']};
const server=http.createServer((req,res)=>{
  if(!hostOK(req.headers.host)){res.writeHead(403);res.end('Loopback Host required');return;}
  const f=files[req.url];if(req.method!=='GET'||!f){res.writeHead(404);res.end();return;}
  res.writeHead(200,{'Content-Type':f[1]+'; charset=utf-8','Cache-Control':'no-store',
    'Referrer-Policy':'no-referrer','X-Frame-Options':'DENY','X-Content-Type-Options':'nosniff',
    'Cross-Origin-Opener-Policy':'same-origin','Content-Security-Policy':
      `default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self' ws://127.0.0.1:${actualPort} ws://localhost:${actualPort}; frame-ancestors 'none'; object-src 'none'`});
  res.end(fs.readFileSync(path.join(root,'web',f[0])));
});
server.maxHeadersCount=20;server.headersTimeout=5000;server.requestTimeout=5000;
const wss=new WebSocketServer({noServer:true,maxPayload:4096,perMessageDeflate:false});
server.on('upgrade',(req,socket,head)=>{
  if(!['/control','/view'].includes(req.url)||!hostOK(req.headers.host)||!originOK(req.headers.origin)){
    socket.end('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n');return;
  }
  socket.setNoDelay(true);wss.handleUpgrade(req,socket,head,ws=>wss.emit('connection',ws,req.url));
});
function toChild(v){
  if(!child||child.stdin.destroyed)return;
  metrics.max_input_buffer=Math.max(metrics.max_input_buffer,child.stdin.writableLength);
  if(child.stdin.writableLength>16384){shutdown('INPUT_BACKPRESSURE');return;}
  child.stdin.write(JSON.stringify(v)+'\n');
}
function send(ws,v,role='control'){
  if(!ws||ws.readyState!==WebSocket.OPEN)return false;
  const text=JSON.stringify(v),size=Buffer.byteLength(text);
  if(role==='control'){
    metrics.max_control_buffer=Math.max(metrics.max_control_buffer,ws.bufferedAmount);
    if(size>4096||ws.bufferedAmount>8192){ws.terminate();return false;}
    metrics.control_bytes+=size;metrics.control_messages++;
  }else{
    if(size>8192||ws.bufferedAmount>16384){ws.terminate();return false;}
    metrics.view_bytes+=size;
  }
  ws.send(text,{compress:false});return true;
}
function revoke(reason){
  if(owner&&owner.opened&&!owner.revoked){owner.revoked=true;
    toChild({kind:'disconnect',connection:owner.id,reason});}
}
function auth(d,type){
  const got=typeof d?.token==='string'?Buffer.from(d.token):Buffer.alloc(0),want=Buffer.from(token);
  return d?.type===type&&got.length===want.length&&crypto.timingSafeEqual(got,want);
}
function viewFrame(v){
  return {mode,state:v.state,reason:String(v.reason||'').slice(0,100),car:v.car,goal:v.goal,reference:v.reference,
    trace:(v.trace||[]).slice(-60),remaining_s:v.remaining_s,software_only:true,physical_output:false};
}
function status(v){
  return {kind:'status',mode,state:v.state,reason:String(v.reason||'').slice(0,200),
    instruction:String(v.instruction||'').slice(0,400),prepare_phase:v.prepare_phase,
    session_requested:v.session_requested,trial_ended:v.trial_ended,remaining_s:v.remaining_s,
    channel:v.channel,gate:v.gate,software_only:true,physical_output:false};
}
wss.on('connection',(ws,route)=>{
  let authenticated=false,linkedOwner=null;
  const timer=setTimeout(()=>ws.close(1008,'Authentication timeout'),1500);
  ws.on('error',()=>{});
  ws.on('message',(bytes,binary)=>{
    if(binary){ws.close(1008,'Text only');return;}
    let d;try{d=JSON.parse(bytes.toString());}catch{ws.close(1008,'JSON invalid');return;}
    if(!authenticated){
      if(route==='/control'){
        if(!auth(d,'hello')||Object.keys(d).sort().join(',')!=='token,type'){ws.close(1008,'Authentication failed');return;}
        if(owner){send(ws,{kind:'denied',reason:'OWNER_EXISTS_OR_ENDED'});ws.close(1008,'Owner exists');return;}
        owner=linkedOwner={id:crypto.randomBytes(12).toString('hex'),control:ws,view:null,opened:false,revoked:false,slot:null,viewSentAt:null};
        owner.pairTimer=setTimeout(()=>{if(!owner.opened){send(ws,{kind:'denied',reason:'VIEW_PAIR_TIMEOUT'});ws.close(1008,'View pair timeout');}},2000);
        send(ws,{kind:'hello',mode,owner:owner.id,view_required:true,software_only:true,physical_output:false});
      }else{
        if(!auth(d,'view_hello')||Object.keys(d).sort().join(',')!=='owner,token,type'
            ||!owner||owner.id!==d.owner||owner.view||owner.revoked||owner.control.readyState!==WebSocket.OPEN){
          ws.close(1008,'View authentication failed');return;
        }
        linkedOwner=owner;owner.view=ws;clearTimeout(owner.pairTimer);
        owner.slot=new ViewSlot(v=>{owner.viewSentAt=steady();send(ws,v,'view');},reason=>ws.close(1008,reason));
        send(ws,{kind:'view_ready',physical_output:false},'view');
        owner.opened=true;toChild({kind:'open',connection:owner.id});
        if(lastState){send(owner.control,status(lastState));owner.slot.offer(viewFrame(lastState));}
      }
      authenticated=true;clearTimeout(timer);return;
    }
    if(route==='/view'){
      if(!d||Object.keys(d).sort().join(',')!=='sequence,type'||d.type!=='ack'){ws.close(1008,'View is read-only');return;}
      if(linkedOwner.viewSentAt!==null&&steady()-linkedOwner.viewSentAt>.100){revoke('VIEW_ACK_TIMEOUT_100MS');return;}
      linkedOwner.slot.ack(d.sequence);
      if(linkedOwner.slot.flight===null)linkedOwner.viewSentAt=null;
      return;
    }
    // Timing samples are server-owned metadata, never accepted as browser authority.
    toChild({kind:'wire',connection:linkedOwner.id,data:d,node_received_steady:steady(),node_event_gap_max_s:maxLoop});
  });
  ws.on('close',()=>{
    clearTimeout(timer);
    if(linkedOwner){clearTimeout(linkedOwner.pairTimer);revoke(route==='/control'?'CONTROL_DISCONNECTED':'VIEW_DISCONNECTED');
      if(route==='/control'&&linkedOwner.view)linkedOwner.view.close(1000,'Control ended');}
  });
});
function shutdown(reason){
  if(ending)return;ending=true;revoke(reason);
  for(const ws of wss.clients)ws.close(1000,reason.slice(0,100));
  if(child&&!child.stdin.destroyed)child.stdin.end();
  server.close();setTimeout(()=>{if(child&&!child.killed)child.kill('SIGTERM');process.exit(0);},900).unref();
}
process.on('SIGINT',()=>shutdown('SERVER_STOP'));process.on('SIGTERM',()=>shutdown('SERVER_STOP'));
server.on('error',e=>{process.stderr.write(String(e)+'\n');shutdown('LISTEN_FAILED');});
server.listen(port,'127.0.0.1',()=>{
  actualPort=server.address().port;
  const env={...process.env,PYTHONDONTWRITEBYTECODE:'1',ROS_DOMAIN_ID:'187',ROS_AUTOMATIC_DISCOVERY_RANGE:'LOCALHOST',ROS_LOG_DIR:path.join(session,'ros_logs')};
  child=spawn('/usr/bin/python3',[path.join(root,'worker.py'),'--mode',mode,'--seconds',String(seconds),'--session',session],{env,cwd:root,stdio:['pipe','pipe','pipe']});
  child.stdin.on('error',()=>{});
  child.stderr.on('data',b=>{fs.appendFileSync(path.join(session,'worker.log'),b);process.stderr.write(b);});
  readline.createInterface({input:child.stdout}).on('line',line=>{
    let v;try{v=JSON.parse(line);}catch{shutdown('WORKER_PROTOCOL');return;}
    if(v.kind==='ready'){
      const url=`http://127.0.0.1:${actualPort}/#${token}`;
      process.stdout.write('BROWSER_READY '+JSON.stringify({mode,port:actualPort,session,url,hardware_entry:false})+'\n');
      process.stdout.write('浏览器地址（完整复制）：'+url+'\n控制/画面分流；严格100ms；本入口没有实车输出。\n');
    }else if(v.kind==='challenge'){
      if(owner&&owner.opened&&!owner.revoked&&v.connection===owner.id)send(owner.control,v);
    }else if(v.kind==='state'){
      lastState=v;if(owner&&owner.opened&&!owner.revoked){send(owner.control,status(v));owner.slot.offer(viewFrame(v));}
    }else if(v.kind==='close'){
      if(owner&&v.connection===owner.id){send(owner.control,{kind:'denied',reason:v.reason});owner.control.close(1008,v.reason);}
    }else if(v.kind==='result'){
      const transport={...metrics,node_event_gap_max_s:maxLoop,view:owner?.slot?.stats()||null,
        separate_control_and_view:true,view_ack_is_authority:false,ssh_outer_stream_shared:true};
      fs.writeFileSync(path.join(session,'transport.json.tmp'),JSON.stringify(transport,null,2)+'\n');
      fs.renameSync(path.join(session,'transport.json.tmp'),path.join(session,'transport.json'));
      if(owner)send(owner.control,{kind:'result',result:{...v.result,last_state:undefined},transport});
      process.stdout.write('BROWSER_RESULT '+JSON.stringify({passed:v.result.passed,completed:v.result.completed,reason:v.result.reason,
        session,physical_output:false})+'\n结果已保存：'+path.join(session,'result.json')+'\n');
    }
  });
  child.on('error',()=>shutdown('WORKER_FAILED'));
  child.on('close',code=>{if(!ending)setTimeout(()=>shutdown('WORKER_ENDED_'+code),400);});
});
