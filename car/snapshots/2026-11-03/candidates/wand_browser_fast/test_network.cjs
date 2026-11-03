'use strict';
const {spawn}=require('child_process'),path=require('path'),fs=require('fs'),http=require('http'),assert=require('assert');
const WS=require('/usr/share/nodejs/ws');
const root=process.env.FAST_OUTPUT||'/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_fast_2026-10-28/prepared';
const delay=ms=>new Promise(r=>setTimeout(r,ms));
async function until(f,ms=3000){const t=Date.now();while(!f()){if(Date.now()-t>ms)throw Error('Network test timeout');await delay(5);}}
async function launch(){
  const p=spawn(process.execPath,[path.join(root,'server.cjs'),'--readonly','--port','0','--seconds','3']);let out='',err='';
  p.stdout.on('data',b=>out+=b);p.stderr.on('data',b=>err+=b);await until(()=>out.includes('BROWSER_READY '));
  return {p,...JSON.parse(out.split('BROWSER_READY ')[1].split('\n')[0]),get errors(){return err;}};
}
async function connect(run,route='/control',origin=`http://127.0.0.1:${run.port}`){
  const ws=new WS(`ws://127.0.0.1:${run.port}${route}`,{origin}),messages=[];ws.on('error',()=>{});
  await new Promise((r,j)=>{ws.once('open',r);ws.once('error',j);});ws.on('message',b=>messages.push(JSON.parse(b)));
  return {ws,messages};
}
async function paired(run,{reply=true,ack=true}={}){
  const c=await connect(run);c.ws.send(JSON.stringify({type:'hello',token:run.url.split('#')[1]}));
  await until(()=>c.messages.some(v=>v.kind==='hello'));const owner=c.messages.find(v=>v.kind==='hello').owner;
  const v=await connect(run,'/view');let seq=0;
  if(ack)v.ws.on('message',b=>{const x=JSON.parse(b);if(x.kind==='view')v.ws.send(JSON.stringify({type:'ack',sequence:x.view_sequence}));});
  if(reply)c.ws.on('message',b=>{const x=JSON.parse(b);if(x.kind==='challenge')c.ws.send(JSON.stringify({type:'pulse',nonce:x.nonce,
    sequence:++seq,focused:true,visible:true,space:false,ui_gap_ms:20,events:[]}));});
  v.ws.send(JSON.stringify({type:'view_hello',token:run.url.split('#')[1],owner}));
  await until(()=>v.messages.some(x=>x.kind==='view'));
  return {c,v,owner};
}
async function result(run){await until(()=>fs.existsSync(path.join(run.session,'result.json')),5000);
  return JSON.parse(fs.readFileSync(path.join(run.session,'result.json')));}
function stop(pair){pair.c.ws.send(JSON.stringify({type:'stop',reason:'TEST_END'}));}
(async()=>{
  const cases=[];
  let run=await launch();let status=await new Promise((resolve,reject)=>http.get({hostname:'127.0.0.1',port:run.port,path:'/',headers:{Host:'evil.example'}},r=>{r.resume();resolve(r.statusCode);}).on('error',reject));
  assert.equal(status,403);await assert.rejects(connect(run,'/control','http://evil.example'));
  let c=await connect(run);c.ws.send(JSON.stringify({type:'hello',token:'wrong'}));await until(()=>c.ws.readyState===WS.CLOSED);
  cases.push('loopback Host Origin and token required');
  let pair=await paired(run);const second=await connect(run);second.ws.send(JSON.stringify({type:'hello',token:run.url.split('#')[1]}));
  await until(()=>second.ws.readyState===WS.CLOSED);assert(second.messages.some(v=>v.kind==='denied'));
  cases.push('single owner and no implicit handover');stop(pair);await result(run);
  run=await launch();pair=await paired(run,{reply:false,ack:true});let r=await result(run);
  assert(r.reason==='INITIAL_BROWSER_PROOF_TIMEOUT'&&r.proofs===0&&!r.authority_ever_acquired);
  cases.push('view ACK alone never creates proof or authority');
  run=await launch();pair=await paired(run,{reply:true,ack:false});r=await result(run);
  assert(r.reason==='VIEW_ACK_TIMEOUT_100MS'&&!r.authority_ever_acquired);cases.push('non-acknowledging view fails100ms independently');
  run=await launch();pair=await paired(run);pair.v.ws.send(JSON.stringify({type:'pulse',held:true}));r=await result(run);
  assert(!r.passed&&r.reason==='VIEW_DISCONNECTED');cases.push('view socket cannot submit control input');
  run=await launch();pair=await paired(run);pair.v.ws.close();r=await result(run);
  assert(r.reason==='VIEW_DISCONNECTED');cases.push('view disconnect revokes while control alive');
  run=await launch();pair=await paired(run);pair.c.ws.close();r=await result(run);
  assert(r.reason==='CONTROL_DISCONNECTED');cases.push('control disconnect revokes');
  run=await launch();pair=await paired(run,{reply:false});
  await until(()=>pair.c.messages.some(v=>v.kind==='challenge'));const nonce=pair.c.messages.find(v=>v.kind==='challenge').nonce;
  const frame={type:'pulse',nonce,sequence:1,focused:true,visible:true,space:false,ui_gap_ms:20,events:[]};
  pair.c.ws.send(JSON.stringify(frame));await delay(5);pair.c.ws.send(JSON.stringify({...frame,sequence:2}));r=await result(run);
  assert(r.reason==='CHALLENGE_EXPIRED_OR_REPLAYED');cases.push('nonce replay rejected on real socket');
  run=await launch();pair=await paired(run,{reply:false});await until(()=>pair.c.messages.some(v=>v.kind==='challenge'));
  pair.c.ws.send(JSON.stringify({type:'pulse',nonce:pair.c.messages.find(v=>v.kind==='challenge').nonce,
    sequence:1,focused:true,visible:true,space:false,ui_gap_ms:20,events:[{kind:'setpoint',x:0,y:0,yaw:0,visible:true}]}));
  r=await result(run);assert(r.reason==='ACTION_NOT_ALLOWED');cases.push('readonly cannot request software or real motion');
  const p=spawn(process.execPath,[path.join(root,'server.cjs'),'--hardware']);p.stdout.resume();p.stderr.resume();
  await new Promise(r=>p.on('exit',r));assert(p.exitCode!==0);cases.push('hardware mode absent');
  const blocked=spawn(process.execPath,[path.join(root,'server.cjs'),'--software']);blocked.stdout.resume();blocked.stderr.resume();
  await new Promise(r=>blocked.on('exit',r));assert(blocked.exitCode!==0);cases.push('unqualified ROS software mode blocked');
  console.log('NETWORK_TEST '+JSON.stringify({passed:true,cases}));
})().catch(e=>{console.error(e);process.exitCode=1;});
