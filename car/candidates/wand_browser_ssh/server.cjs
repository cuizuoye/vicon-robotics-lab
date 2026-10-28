'use strict';
// Fixed loopback listener + authenticated single browser. Never actuates.
const http=require('http'),fs=require('fs'),path=require('path'),crypto=require('crypto');
const {spawn}=require('child_process'),readline=require('readline');
const {WebSocketServer,WebSocket}=require('/usr/share/nodejs/ws');
const root=__dirname,args=process.argv.slice(2);let mode='readonly',port=8765,seconds=60;
for(let i=0;i<args.length;i++){
  if(args[i]==='--readonly')mode='readonly';
  else if(args[i]==='--software')mode='software';
  else if(args[i]==='--port')port=Number(args[++i]);
  else if(args[i]==='--seconds')seconds=Number(args[++i]);
  else throw Error('Unknown/forbidden option: '+args[i]+'; hardware entry is not implemented');
}
if(!Number.isInteger(port)||port<0||port>65535||!Number.isFinite(seconds)||seconds<1||seconds>120)throw Error('Bad port/duration');
const sessions=path.join(root,'sessions');fs.mkdirSync(sessions,{recursive:true,mode:0o700});
const session=fs.mkdtempSync(path.join(sessions,'browser_'+mode+'_'));fs.chmodSync(session,0o700);
const token=crypto.randomBytes(24).toString('hex'),connections=new Map();let child,ending=false,actualPort=port,lastState=null;
const mime={'/':'text/html; charset=utf-8','/app.js':'text/javascript; charset=utf-8','/style.css':'text/css; charset=utf-8'};
function allowedHost(host){return host===`127.0.0.1:${actualPort}`||host===`localhost:${actualPort}`;}
function allowedOrigin(origin){return origin===`http://127.0.0.1:${actualPort}`||origin===`http://localhost:${actualPort}`;}
const server=http.createServer((req,res)=>{
  if(!allowedHost(req.headers.host)){res.writeHead(403);res.end('Loopback Host required');return;}
  const name=req.url;
  if(req.method!=='GET'||!Object.hasOwn(mime,name)){res.writeHead(404);res.end('Not found');return;}
  res.writeHead(200,{'Content-Type':mime[name],'Cache-Control':'no-store','X-Frame-Options':'DENY',
    'Referrer-Policy':'no-referrer','X-Content-Type-Options':'nosniff','Cross-Origin-Opener-Policy':'same-origin',
    'Content-Security-Policy':`default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self' ws://127.0.0.1:${actualPort} ws://localhost:${actualPort}; frame-ancestors 'none'; object-src 'none'`});
  res.end(fs.readFileSync(path.join(root,'web',name==='/'?'index.html':name.slice(1))));
});
server.maxHeadersCount=16;server.headersTimeout=5000;server.requestTimeout=5000;
const wss=new WebSocketServer({noServer:true,maxPayload:4096,perMessageDeflate:false});
server.on('upgrade',(req,socket,head)=>{
  if(req.url!=='/control'||!allowedHost(req.headers.host)||!allowedOrigin(req.headers.origin)){
    socket.end('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n');return;
  }
  socket.setNoDelay(true);wss.handleUpgrade(req,socket,head,ws=>wss.emit('connection',ws,req));
});
function sendToChild(value){
  if(!child||child.stdin.destroyed)return;
  if(child.stdin.writableLength>32768){shutdown('INPUT_BACKPRESSURE');return;}
  child.stdin.write(JSON.stringify(value)+'\n');
}
function safeSend(ws,value){
  if(ws.readyState!==WebSocket.OPEN)return;
  if(ws.bufferedAmount>65536){ws.terminate();return;}
  ws.send(JSON.stringify(value));
}
wss.on('connection',ws=>{
  const id=crypto.randomBytes(12).toString('hex');let authenticated=false;
  const timer=setTimeout(()=>ws.close(1008,'Authentication timeout'),1000);
  ws.on('error',()=>{});
  ws.on('message',(bytes,binary)=>{
    if(binary){ws.close(1008,'Text only');return;}
    let data;try{data=JSON.parse(bytes.toString());}catch{ws.close(1008,'Bad JSON');return;}
    if(!authenticated){
      const supplied=typeof data?.token==='string'?Buffer.from(data.token):Buffer.alloc(0),expected=Buffer.from(token);
      if(data?.type!=='hello'||Object.keys(data).sort().join(',')!=='token,type'||supplied.length!==expected.length||!crypto.timingSafeEqual(supplied,expected)){
        ws.close(1008,'Authentication failed');return;
      }
      authenticated=true;clearTimeout(timer);connections.set(id,ws);
      sendToChild({kind:'open',connection:id});
      safeSend(ws,{kind:'hello',mode,software_only:true,physical_output:false});
      if(lastState)safeSend(ws,lastState);
    }else sendToChild({kind:'wire',connection:id,data});
  });
  ws.on('close',()=>{clearTimeout(timer);if(authenticated){connections.delete(id);sendToChild({kind:'disconnect',connection:id});}});
});
function broadcast(value){for(const ws of connections.values())safeSend(ws,value);}
function shutdown(reason){
  if(ending)return;ending=true;
  for(const ws of connections.values())ws.close(1000,reason.slice(0,100));
  if(child&&!child.stdin.destroyed)child.stdin.end();
  server.close();setTimeout(()=>{if(child&&!child.killed)child.kill('SIGTERM');process.exit(0);},800).unref();
}
process.on('SIGINT',()=>shutdown('SERVER_STOP'));process.on('SIGTERM',()=>shutdown('SERVER_STOP'));
server.on('error',error=>{process.stderr.write(String(error)+'\n');shutdown('LISTEN_FAILED');});
server.listen(port,'127.0.0.1',()=>{
  actualPort=server.address().port;
  const env={...process.env,PYTHONDONTWRITEBYTECODE:'1',ROS_DOMAIN_ID:'187',ROS_AUTOMATIC_DISCOVERY_RANGE:'LOCALHOST',ROS_LOG_DIR:path.join(session,'ros_logs')};
  child=spawn('/usr/bin/python3',[path.join(root,'worker.py'),'--mode',mode,'--seconds',String(seconds),'--session',session],{cwd:root,env,stdio:['pipe','pipe','pipe']});
  child.stdin.on('error',()=>{});
  child.stderr.on('data',b=>{fs.appendFileSync(path.join(session,'worker.log'),b);process.stderr.write(b);});
  readline.createInterface({input:child.stdout}).on('line',line=>{
    let value;try{value=JSON.parse(line);}catch{process.stderr.write('Worker protocol error\n');shutdown('WORKER_PROTOCOL');return;}
    if(value.kind==='ready'){
      const url=`http://127.0.0.1:${actualPort}/#${token}`;
      process.stdout.write('BROWSER_READY '+JSON.stringify({mode,port:actualPort,session,url,hardware_entry:false})+'\n');
      process.stdout.write('仅监听127.0.0.1。Windows浏览器本地绘图；无需X11。先SSH转发同一端口，再打开上面的地址。\n');
      process.stdout.write('浏览器地址（完整复制）：'+url+'\n');
      const host=(process.env.SSH_CONNECTION||'').split(/\s+/)[2];
      if(host&&require('net').isIP(host))process.stdout.write(`Windows本地终端另开：ssh -N -T -o ExitOnForwardFailure=yes -L 127.0.0.1:${actualPort}:127.0.0.1:${actualPort} czy@${host}\n`);
    }else if(value.kind==='challenge'){
      const ws=connections.get(value.connection);if(ws)safeSend(ws,value);
    }else if(value.kind==='close'){
      const ws=connections.get(value.connection);if(ws){safeSend(ws,{kind:'denied',reason:value.reason});ws.close(1008,value.reason);}
    }else if(value.kind==='state'){lastState=value;broadcast(value);}
    else if(value.kind==='result'){
      broadcast(value);process.stdout.write('BROWSER_RESULT '+JSON.stringify(value.result)+'\n');
      process.stdout.write('结果已保存：'+path.join(session,'result.json')+'\n');
    }
  });
  child.on('error',e=>{process.stderr.write(String(e)+'\n');shutdown('WORKER_START_FAILED');});
  child.on('close',code=>{if(!ending){process.stdout.write('BROWSER_WORKER_EXIT '+code+'\n');setTimeout(()=>shutdown('WORKER_ENDED'),400);}});
});
