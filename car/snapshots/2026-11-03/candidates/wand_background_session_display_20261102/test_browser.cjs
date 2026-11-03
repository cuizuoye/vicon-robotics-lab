'use strict';
// Real browser, ROS and exact native-C loopback; never addresses the real robot.
const fs=require('fs'),path=require('path'),assert=require('assert'),http=require('http'),{spawn}=require('child_process');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(process.argv[2]),out=path.resolve(process.argv[3]),name=process.argv[4]||'background';
const seconds=Number(process.argv[5]||40),sleep=ms=>new Promise(r=>setTimeout(r,ms));
async function until(fn,timeout=30000){const started=Date.now();while(!await fn()){if(Date.now()-started>timeout)throw Error('timeout');await sleep(20);}}
function lastJSON(file){try{const text=fs.readFileSync(file,'utf8');const lines=text.slice(0,text.lastIndexOf('\n')).trim().split('\n');return JSON.parse(lines.at(-1));}catch{return null;}}
function httpStop(info,reason='LOCAL_STOP',token=info.url.split('#')[1],origin=`http://127.0.0.1:${info.port}`){
 return new Promise((resolve,reject)=>{const data=JSON.stringify({type:'stop',token,reason});
  const req=http.request({hostname:'127.0.0.1',port:info.port,path:'/operator-stop',method:'POST',headers:{Origin:origin,'Content-Type':'application/json','Content-Length':Buffer.byteLength(data)}},r=>{r.resume();r.on('end',()=>resolve(r.statusCode));});
  req.on('error',reject);req.end(data);
 });
}
async function main(){
 fs.mkdirSync(out,{recursive:true});assert.equal(process.env.WAND_4WSC_MODE,'rehearsal');
 const p=JSON.parse(fs.readFileSync(process.env.WAND_4WSC_PROFILE));assert.equal(p.hardware.esp_ip,'127.0.0.1');assert.equal(p.hardware.network_interface,'lo');
 for(const port of [p.hardware.command_port,p.hardware.telemetry_port])assert(![46000,46001].includes(port));
 const child=spawn(process.execPath,[path.join(root,'server.cjs'),'--rehearsal',name==='occlusion'?'--test-occlusion':'--synthetic','--seconds',String(seconds)],{env:process.env});
 let stdout='',stderr='',browser,page,client,info,result,frozen=false,stoppedPID=null;const errors=[];
 child.stdout.on('data',b=>stdout+=b);child.stderr.on('data',b=>stderr+=b);
 try{
  await until(()=>stdout.includes('BROWSER_READY '));info=JSON.parse(stdout.split('BROWSER_READY ')[1].split('\n')[0]);assert(!info.hardware_entry);
  const headed=['background','local_stop','timer'].includes(name);
  browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:!headed,args:['--disable-gpu'],
    ignoreDefaultArgs:['--disable-background-timer-throttling','--disable-backgrounding-occluded-windows','--disable-renderer-backgrounding']});
  const context=await browser.newContext({viewport:{width:1280,height:720}});page=await context.newPage();
  page.on('pageerror',e=>errors.push(String(e)));
  // Initial foreground is a controlled browser-test fixture. Disable its focus
  // emulation BEFORE all background/minimize observations below; never override
  // document.hasFocus/document.hidden in application code.
  client=await context.newCDPSession(page);
  await page.goto(info.url);await page.bringToFront();
  await page.evaluate(()=>{document.querySelector('h1').textContent='离线软件测试 · 不连接实车';});
  await until(()=>page.evaluate(()=>latest?.prepare_phase==='READY'&&!!offer),20000);
  const trace=path.join(info.session,'TRACE.jsonl'),file=path.join(info.session,'result.json');
  const noStart=name==='no_start';let backgroundStarted=null,backgroundEnded=null,freeze_s=0,unfocused_s=0,minimized_s=0,pageHiddenReported=false;
  if(noStart){
   client=await context.newCDPSession(page);await client.send('Emulation.setFocusEmulationEnabled',{enabled:false});
   const other=await context.newPage();await (await context.newCDPSession(other)).send('Emulation.setFocusEmulationEnabled',{enabled:false});
   await other.goto('about:blank');await other.bringToFront();await sleep(1500);
   assert.equal(lastJSON(trace)?.status.state,'ARMED');await page.bringToFront();await page.keyboard.press('Escape');
  }else{
   await page.keyboard.press('Space');await until(()=>lastJSON(trace)?.status.state==='ACTIVE',7000);
   if(name==='background'||name==='local_stop'||name==='timer'){
    client=await context.newCDPSession(page);
    await client.send('Emulation.setFocusEmulationEnabled',{enabled:false});
    const other=await context.newPage();await (await context.newCDPSession(other)).send('Emulation.setFocusEmulationEnabled',{enabled:false});
    await other.goto('about:blank');await other.bringToFront();
    backgroundStarted=lastJSON(trace).t;
    const visibility=await page.evaluate(()=>({hidden:document.hidden,focused:document.hasFocus()}));
    assert(!visibility.focused,'test must really unfocus page');pageHiddenReported=visibility.hidden;
    const period=name==='background'?5000:1200;await sleep(period);unfocused_s=period/1000;
    assert.equal(lastJSON(trace)?.status.state,'ACTIVE','switching tabs revoked the session');
    if(name==='background'){
     await page.bringToFront();const win=await client.send('Browser.getWindowForTarget');
     await client.send('Browser.setWindowBounds',{windowId:win.windowId,bounds:{windowState:'minimized'}});await sleep(200);
     assert.equal((await client.send('Browser.getWindowBounds',{windowId:win.windowId})).bounds.windowState,'minimized');
     assert(!await page.evaluate(()=>document.hasFocus()),'minimized window must really lose focus');
     await sleep(3000);minimized_s=3;assert.equal(lastJSON(trace)?.status.state,'ACTIVE','minimizing revoked session');
     await client.send('Browser.setWindowBounds',{windowId:win.windowId,bounds:{windowState:'normal'}});
     await other.bringToFront();
    }
    await client.send('Page.setWebLifecycleState',{state:'frozen'});frozen=true;
    const freeze=name==='background'?5000:name==='timer'?(seconds+1)*1000:1500;
    await sleep(freeze);freeze_s=freeze/1000;
    if(name!=='timer')assert.equal(lastJSON(trace)?.status.state,'ACTIVE','frozen renderer revoked the session');
    backgroundEnded=lastJSON(trace).t;
    if(name==='local_stop'){
     assert.equal(await httpStop(info,'LOCAL_STOP','wrong-token'),403);
     assert.equal(lastJSON(trace)?.status.state,'ACTIVE','bad token must not mutate session');
     assert.equal(await httpStop(info,'LOCAL_STOP',undefined,'http://evil'),403);
     assert.equal(await httpStop(info),202);
    }
    await client.send('Page.setWebLifecycleState',{state:'active'});frozen=false;
    await page.bringToFront();
    if(name==='background'){
     await until(()=>page.evaluate(()=>latest?.state==='ACTIVE'),3000);
     assert(!await page.evaluate(()=>ended));await page.setViewportSize({width:1920,height:1080});await sleep(600);
     await page.keyboard.press('Escape');
    }
   }else if(name==='renderer_stall'){
    await page.evaluate(()=>{const t=performance.now();while(performance.now()-t<350){}});
    await sleep(800);assert.equal(lastJSON(trace)?.status.state,'ACTIVE');await page.keyboard.press('Escape');
   }else if(name==='node_stall'||name==='worker_stall'){
    stoppedPID=name==='node_stall'?child.pid:Number(fs.readFileSync(`/proc/${child.pid}/task/${child.pid}/children`,'utf8').trim().split(/\s+/)[0]);
    assert(stoppedPID>1);process.kill(stoppedPID,'SIGSTOP');await sleep(400);process.kill(stoppedPID,'SIGCONT');stoppedPID=null;
   }else if(name==='close')await page.close();
   else if(name==='second_space')await page.keyboard.press('Space');
   else if(name==='estop')await page.locator('#estop').click();
   else if(name==='occlusion'){} // The synthetic Sign becomes invisible at2.5s.
   else throw Error('unknown test case');
  }
  await until(()=>fs.existsSync(file),20000);
  const r=JSON.parse(fs.readFileSync(file));assert.equal(r.ros_domain,196);assert(!r.hardware_socket_opened&&!r.physical_motion_started);
  assert.equal(r.authority_ever_acquired,!noStart);assert(r.stop_evidence.gate_output_zero);
  const native=r.stop_evidence.exact_C_peer.final_sample.status;
  assert(!native.permitted&&native.raw_duty.every(v=>v===0)&&native.target_cps.every(v=>v===0),'native C outputs must actually be zero');
  if(name==='estop'){
   // ESTOP deliberately invalidates normal feedback. Never relabel a latched
   // zero-output emergency as a confirmed physical stop or make it restartable.
   assert.equal(r.stop_evidence.stop_confirmed,false);
   const tail=lastJSON(trace);assert(tail.gate.hard_fault&&tail.gate.fault==='ESTOP');
  }else assert(r.stop_evidence.stop_confirmed,'normal/fault stop confirmation missing');
  assert.equal(r.accepted_commands,noStart?0:1);assert.equal(r.proofs,0,'must not fabricate UI proofs');
  if(['background','renderer_stall','no_start'].includes(name))assert.equal(r.reason,'CLIENT_ESC');
  if(name==='local_stop')assert.equal(r.reason,'LOCAL_OPERATOR_STOP');
  if(name==='node_stall')assert.equal(r.reason,'BACKEND_COMMAND_CHANNEL_TIMEOUT');
  if(name==='worker_stall')assert.equal(r.reason,'SERVER_CONTROL_LOOP_GAP');
  if(name==='second_space')assert.equal(r.reason,'CLIENT_SECOND_SPACE');
  if(name==='estop')assert.equal(r.reason,'OPERATOR_ESTOP');
  if(name==='occlusion')assert(/WAND_POSE_TIMEOUT|car or wand pose timeout/.test(r.reason),r.reason);
  if(name==='timer')assert(r.completed&&r.passed,r.reason);
  let backgroundPath=0;
  if(backgroundStarted!==null){
   const rows=fs.readFileSync(path.join(info.session,'POSE_OBSERVATIONS.jsonl'),'utf8').trim().split('\n').map(JSON.parse)
    .filter(v=>v.at>=backgroundStarted&&v.at<=backgroundEnded);
   for(let i=1;i<rows.length;i++)backgroundPath+=Math.hypot(rows[i].x-rows[i-1].x,rows[i].y-rows[i-1].y);
   assert(backgroundPath>.003,'software car did not move during hidden/frozen interval');
  }
  if(!page.isClosed()){
   const expected=name==='estop'?'STOP_UNCONFIRMED':'STOPPED_CONFIRMED';
   await until(()=>page.evaluate(expected=>resultSeen&&latest?.state===expected,expected),4000);
   await sleep(250);assert.equal(await page.evaluate(()=>latest.state),expected,'late view overwrote terminal result');
  }
  await sleep(150);
  const transport=JSON.parse(fs.readFileSync(path.join(info.session,'transport.json')));assert(transport.view.max_pending<=1);
  assert.equal(errors.length,0);if(!page.isClosed())await page.screenshot({path:path.join(out,'page.png')});
  result={passed:true,case:name,session:info.session,reason:r.reason,actual_browser:true,actual_ROS_native_C_loopback:true,
   headful_browser:headed,initial_focus_is_browser_fixture:true,background_focus_emulation_disabled:['background','local_stop','timer','no_start'].includes(name),unfocused_s,page_hidden_reported:pageHiddenReported,minimized_s,frozen_renderer_s:freeze_s,background_software_car_path_m:backgroundPath,
   accepted_commands:r.accepted_commands,browser_proofs:r.proofs,max_pending_view_frames:transport.view.max_pending,
   stop_confirmed:r.stop_evidence.stop_confirmed,native_C_output_zero:true,emergency_remains_locked:name==='estop',
   physical_output:false,firmware_changed:false,network_settings_changed:false,errors};
 }catch(e){result={passed:false,case:name,error:e.stack,info,errors};throw e;}
 finally{
  if(stoppedPID){try{process.kill(stoppedPID,'SIGCONT');}catch{}}
  if(frozen&&client){try{await client.send('Page.setWebLifecycleState',{state:'active'});}catch{}}
  fs.writeFileSync(path.join(out,'server.stdout.log'),stdout);fs.writeFileSync(path.join(out,'server.stderr.log'),stderr);
  fs.writeFileSync(path.join(out,'RESULT.json'),JSON.stringify(result,null,2)+'\n');
  if(browser)await browser.close();if(child.exitCode===null){child.kill('SIGTERM');await sleep(800);if(child.exitCode===null)child.kill('SIGKILL');}
 }
 console.log(JSON.stringify(result));
}
main().catch(e=>{console.error(e);process.exitCode=1;});
