'use strict';
// Real headless Chromium + real control/view WS + ROS + native C loopback peer.
// Never sets hasFocus, bypasses InputGuard, or uses a physical UDP endpoint.
const fs=require('fs'),path=require('path'),assert=require('assert'),{spawn}=require('child_process');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(process.argv[2]),out=path.resolve(process.argv[3]),name=process.argv[4]||'idle';
const delay=ms=>new Promise(r=>setTimeout(r,ms));
async function until(fn,ms=35000){const start=Date.now();while(!await fn()){if(Date.now()-start>ms)throw Error('timeout');await delay(20);}}
async function main(){
 fs.mkdirSync(out,{recursive:true});
 assert.equal(process.env.WAND_4WSC_MODE,'rehearsal');
 const profile=JSON.parse(fs.readFileSync(process.env.WAND_4WSC_PROFILE));
 assert.equal(profile.hardware.esp_ip,'127.0.0.1');assert.equal(profile.hardware.network_interface,'lo');
 assert(![46000,46001].includes(profile.hardware.command_port));assert(![46000,46001].includes(profile.hardware.telemetry_port));
 const child=spawn(process.execPath,[path.join(root,'server.cjs'),'--rehearsal','--synthetic','--seconds','30'],{env:process.env});
 let stdout='',stderr='',browser,page,info,result;const errors=[];
 child.stdout.on('data',b=>stdout+=b);child.stderr.on('data',b=>stderr+=b);
 try{
  await until(()=>stdout.includes('BROWSER_READY '));info=JSON.parse(stdout.split('BROWSER_READY ')[1].split('\n')[0]);
  assert(!info.hardware_entry);
  browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--disable-gpu']});
  page=await browser.newPage({viewport:{width:1536,height:747}});
  page.on('pageerror',e=>errors.push(String(e)));
  await page.addInitScript(()=>{
   window.__timing={tasks:[],maxRAF:0,frames:0};let previous=null;
   try{new PerformanceObserver(list=>{for(const v of list.getEntries())window.__timing.tasks.push({at:v.startTime,ms:v.duration});}).observe({type:'longtask',buffered:true});}catch{}
   function frame(now){if(previous!==null)window.__timing.maxRAF=Math.max(window.__timing.maxRAF,now-previous);previous=now;window.__timing.frames++;requestAnimationFrame(frame);}requestAnimationFrame(frame);
  });
  await page.goto(info.url);await page.bringToFront();
  await until(()=>page.evaluate(()=>typeof latest!=='undefined'&&latest?.prepare_phase==='READY'),20000);
  assert(await page.evaluate(()=>document.hasFocus()&&!document.hidden));
  if(name!=='idle'){
   await page.keyboard.press('Space');
   await until(()=>page.evaluate(()=>latest?.state==='ACTIVE'),5000);
  }
  if(name==='stall'){
   await page.evaluate(()=>{const start=performance.now();while(performance.now()-start<180){/* deliberate MAIN-thread failure */}});
  }else{
   const count=Number(process.env.BROWSER_TEST_SECONDS||8);
   for(let i=0;i<count;i++){
    if(name==='resize')await page.setViewportSize(i%2?{width:1536,height:747}:{width:1920,height:1080});
    await delay(1000);
    assert(!await page.evaluate(()=>ended),'client stopped early: '+await page.locator('#connection').innerText());
   }
   await page.keyboard.press('Escape');
  }
  await until(()=>fs.existsSync(path.join(info.session,'result.json')),15000);
  const r=JSON.parse(fs.readFileSync(path.join(info.session,'result.json')));
  assert(!r.hardware_socket_opened&&!r.physical_motion_started);assert.equal(r.ros_domain,196);
  assert(r.stop_evidence.stop_confirmed&&r.stop_evidence.gate_output_zero);
  assert.equal(r.authority_ever_acquired,name!=='idle');
  if(name==='stall')assert(/LOOP_GAP|PROOF_TIMEOUT|ACK_TIMEOUT|STALE_100MS/.test(r.reason),r.reason);
  else assert.equal(r.reason,'CLIENT_ESC');
  const timing=await page.evaluate(()=>({...window.__timing,focused:document.hasFocus(),clientMaxGap,replyCount,maxReplyDispatch,maxSendCall}));
  await page.screenshot({path:path.join(out,'page.png')});
  assert.equal(errors.length,0);
  result={passed:true,case:name,actual_browser:true,actual_control_view_websockets:true,InputGuard_active:true,
   actual_ROS_native_C_loopback:true,session:info.session,reason:r.reason,proofs:r.proofs,max_proof_age_s:r.max_proof_age_s,
   max_control_gap_s:r.max_control_gap_s,max_client_gap_s:r.max_client_gap_s,timing,errors,
   physical_output:false,firmware_changed:false,network_settings_changed:false};
 }catch(e){result={passed:false,case:name,error:e.stack,info,errors};throw e;}
 finally{
  fs.writeFileSync(path.join(out,'server.stdout.log'),stdout);fs.writeFileSync(path.join(out,'server.stderr.log'),stderr);
  fs.writeFileSync(path.join(out,'RESULT.json'),JSON.stringify(result,null,2)+'\n');
  if(browser)await browser.close();
  if(child.exitCode===null){child.kill('SIGTERM');await delay(700);if(child.exitCode===null)child.kill('SIGKILL');}
 }
 console.log(JSON.stringify(result));
}
main().catch(e=>{console.error(e);process.exitCode=1;});
