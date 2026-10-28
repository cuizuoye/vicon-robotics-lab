'use strict';
const {spawn}=require('child_process'),http=require('http'),path=require('path'),fs=require('fs'),assert=require('assert');
const WS=require('/usr/share/nodejs/ws');
const ready=process.env.BROWSER_OUTPUT||'/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_ssh_2026-10-27/prepared';
const delay=ms=>new Promise(r=>setTimeout(r,ms));
async function launch(){
  const p=spawn(process.execPath,[path.join(ready,'server.cjs'),'--readonly','--port','0','--seconds','2']);let out='';
  p.stdout.on('data',b=>out+=b);p.stderr.on('data',b=>process.stderr.write(b));
  for(let n=0;n<500&&!out.includes('BROWSER_READY ');n++)await delay(10);
  assert(out.includes('BROWSER_READY '),out);return {p,...JSON.parse(out.split('BROWSER_READY ')[1].split('\n')[0])};
}
async function connect(run,token,origin=`http://127.0.0.1:${run.port}`){
  const ws=new WS(`ws://127.0.0.1:${run.port}/control`,{origin});
  ws.on('error',()=>{});await new Promise((resolve,reject)=>{ws.once('open',resolve);ws.once('error',reject);});
  const messages=[];ws.on('message',b=>messages.push(JSON.parse(b)));
  ws.send(JSON.stringify({type:'hello',token:token===undefined?run.url.split('#')[1]:token}));
  return {ws,messages};
}
async function waitFor(f,limit=1500){const t=Date.now();while(!f()){if(Date.now()-t>limit)throw Error('Test timeout');await delay(10);}}
async function get(run,host){return new Promise((resolve,reject)=>http.get({hostname:'127.0.0.1',port:run.port,path:'/',headers:{Host:host}},r=>{r.resume();resolve(r.statusCode);}).on('error',reject));}
function respond(c,msg,sequence=1,events=[]){c.ws.send(JSON.stringify({type:'pulse',nonce:msg.nonce,sequence,focused:true,visible:true,space:false,ui_gap_ms:20,events}));}
async function report(run){await waitFor(()=>fs.existsSync(path.join(run.session,'result.json')),3000);return JSON.parse(fs.readFileSync(path.join(run.session,'result.json')));}
(async()=>{
  const cases=[];
  let run=await launch();assert.equal(await get(run,`127.0.0.1:${run.port}`),200);
  assert.equal(await get(run,'evil.example'),403);
  await assert.rejects(connect(run,undefined,'http://evil.example'));
  let c=await connect(run,'bad');await waitFor(()=>c.ws.readyState===WS.CLOSED);cases.push('Host-Origin-token rejection');
  c=await connect(run);await waitFor(()=>c.messages.some(v=>v.kind==='challenge'));
  const second=await connect(run);await waitFor(()=>second.ws.readyState===WS.CLOSED);
  assert(second.messages.some(v=>v.kind==='denied'));cases.push('single-owner exclusion');
  const challenge=c.messages.find(v=>v.kind==='challenge');respond(c,challenge);await delay(10);respond(c,challenge,2);
  let r=await report(run);assert(r.fault==='CHALLENGE_EXPIRED_OR_REPLAYED'&&!r.ros_initialized&&!r.authority_ever_acquired);
  cases.push('network nonce replay rejected');
  run=await launch();c=await connect(run);await waitFor(()=>c.messages.some(v=>v.kind==='challenge'));
  respond(c,c.messages.find(v=>v.kind==='challenge'),1,[{kind:'setpoint',x:.2,y:0,yaw:0,visible:true}]);
  r=await report(run);assert(r.fault==='ACTION_NOT_ALLOWED');cases.push('readonly cannot control');
  run=await launch();c=await connect(run);await waitFor(()=>c.messages.some(v=>v.kind==='challenge'));c.ws.close();
  r=await report(run);assert(r.fault==='BROWSER_DISCONNECTED');cases.push('disconnect irrevocably stops');
  run=await launch();c=await connect(run);await waitFor(()=>c.messages.some(v=>v.kind==='challenge'));
  c.ws.send('x'.repeat(5000));r=await report(run);assert(r.fault==='BROWSER_DISCONNECTED');cases.push('payload bound');
  const denied=spawn(process.execPath,[path.join(ready,'server.cjs'),'--hardware']);
  denied.stdout.resume();denied.stderr.resume();await new Promise(r=>denied.on('exit',r));assert(denied.exitCode!==0);cases.push('hardware CLI absent');
  console.log('NETWORK_TEST_RESULT '+JSON.stringify({passed:true,cases}));
})().catch(e=>{console.error(e);process.exitCode=1;});
