'use strict';
// Actual Chrome + actual private ROS nodes. No hardware entry exists.
const {chromium}=require('/home/PRIMARY_USER/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const {spawn}=require('child_process'),fs=require('fs'),path=require('path'),assert=require('assert');
const ready=process.env.BROWSER_OUTPUT||'/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_ssh_2026-10-27/prepared';
const results=[];
const delay=ms=>new Promise(r=>setTimeout(r,ms));
async function launch(mode,seconds=10){
  const p=spawn(process.execPath,[path.join(ready,'server.cjs'),'--'+mode,'--port','0','--seconds',String(seconds)],{env:process.env});
  let output='',errors='';p.stdout.on('data',b=>output+=b);p.stderr.on('data',b=>errors+=b);
  const start=Date.now();while(!output.includes('BROWSER_READY ')){
    if(p.exitCode!==null||Date.now()-start>15000)throw Error('Server start failed '+output+errors);
    await delay(20);
  }
  const info=JSON.parse(output.split('BROWSER_READY ')[1].split('\n')[0]);
  return {p,info,get output(){return output;},get errors(){return errors;}};
}
async function result(run){
  const start=Date.now();while(!fs.existsSync(path.join(run.info.session,'result.json'))){
    if(Date.now()-start>5000)throw Error('No result '+run.output+run.errors);await delay(25);
  }
  return JSON.parse(fs.readFileSync(path.join(run.info.session,'result.json')));
}
async function pageFor(browser,run){
  const context=await browser.newContext({viewport:{width:1280,height:720}}),page=await context.newPage();
  const errors=[];page.on('pageerror',e=>errors.push(String(e)));
  await page.goto(run.info.url);await page.bringToFront();await page.click('h1');
  await page.waitForFunction(()=>document.getElementById('status').textContent.includes('状态：'));
  return {context,page,errors};
}
async function state(page){return page.evaluate(()=>latest);}
async function setpoint(page,x,y,yaw=0,visible=true){
  await page.evaluate(v=>{for(const k of ['x','y','yaw'])document.getElementById(k).value=v[k];
    document.getElementById('visible').checked=v.visible;document.getElementById('x').dispatchEvent(new Event('input'));},
    {x,y,yaw,visible});
}
async function active(browser,seconds=45){
  const run=await launch('software',seconds),ui=await pageFor(browser,run),{page}=ui;
  await page.waitForFunction(()=>latest?.prepare_phase==='READY',{},{timeout:12000});
  assert(!(await state(page)).session_requested,'auto prepare must not enable');
  await setpoint(page,.2,.1,20);await delay(700); // Adjust rod AFTER preparation, BEFORE enable.
  await page.keyboard.press('Space');
  await page.waitForFunction(()=>latest?.state==='ACTIVE',{},{timeout:4000});
  await delay(200);let first=await state(page);
  assert(Math.hypot(first.car.x-12,first.car.y+8)<.01,'enable origin must be neutral');
  await page.screenshot({path:path.join(run.info.session,'software_1280x720.png')});
  return {run,...ui};
}
(async()=>{
  const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-background-timer-throttling']});
  try{
    if(!process.env.SOFTWARE_ONLY){
    // A complete readonly run: actual keys recorded, no ROS or authority.
    let run=await launch('readonly',Number(process.env.READONLY_SECONDS||8)),ui=await pageFor(browser,run);
    await ui.page.keyboard.press('Space');
    await ui.page.screenshot({path:path.join(run.info.session,'browser_1280x720.png')});
    await ui.page.waitForFunction(()=>resultSeen,{},{timeout:(Number(process.env.READONLY_SECONDS||8)+5)*1000});
    let r=await result(run);assert(r.passed&&r.completed&&!r.ros_initialized&&!r.hardware_socket_opened&&!r.authority_ever_acquired);
    assert(r.key_events.press===1&&r.key_events.release===1);assert.deepEqual(ui.errors,[]);
    results.push({case:'readonly_timer_actual_browser',...r,session:run.info.session});await ui.context.close();
    // A stalled MAIN thread cannot be kept alive by the socket/server threads.
    run=await launch('readonly');ui=await pageFor(browser,run);
    await ui.page.evaluate(()=>{const end=performance.now()+265;while(performance.now()<end){}});
    r=await result(run);assert(!r.passed&&r.fault&&r.proofs>0&&!r.authority_ever_acquired);
    results.push({case:'main_thread265ms_revokes',...r});await ui.context.close();
    // Tab hidden, no background lease and no reconnect/resume.
    run=await launch('readonly');ui=await pageFor(browser,run);
    const other=await ui.context.newPage();await other.goto('about:blank');await other.bringToFront();
    // Chromium headless can keep both pages notionally visible: actual browser focus event still invokes handler.
    await ui.page.evaluate(()=>window.dispatchEvent(new Event('blur')));
    r=await result(run);assert(!r.passed&&r.fault.includes('WINDOW_'));
    results.push({case:'focus_loss_revokes',...r});await ui.context.close();
    }
    // Actual ROS XY/yaw and reversal with the unchanged reviewed servo.
    let sim=await active(browser);let page=sim.page;
    await setpoint(page,.4,.2,40);await delay(900);let s=await state(page);
    assert(s.state==='ACTIVE',JSON.stringify(s));assert(s.car.x>12.06&&s.car.y>-7.99,'XY follow');
    assert(s.car.yaw>.01,'yaw follow');
    assert(s.goal&&Math.abs(s.goal.x-12.2)<.02&&Math.abs(s.goal.y+7.9)<.02,'XY1:1');
    assert(Math.abs(s.goal.yaw-20*Math.PI/180)<.02,'yaw1:1 relative');
    await setpoint(page,.0,.0,0);await delay(1100);s=await state(page);
    assert(s.state==='ACTIVE'&&s.goal.x<12.01&&s.car.x<12.10,'reverse');
    await page.keyboard.press('Space');await delay(250);s=await state(page);
    assert(!s.session_requested&&s.trial_ended,'second Space stops');
    assert(!s.gate||s.gate.output_zero,'gate zero');
    await page.keyboard.press('Space');await delay(150);assert(!(await state(page)).session_requested,'no auto/repeated resume');
    await page.keyboard.press('Escape');let r=await result(sim.run);
    assert(r.ros_initialized&&r.ros_domain===187&&!r.hardware_bridge_constructed&&!r.hardware_socket_opened);
    assert(r.software_stop_evidence.gate_zero_observed&&!r.software_stop_evidence.held);
    results.push({case:'actual_ros_xy_yaw_reverse_second_space',...r,session:sim.run.info.session});await sim.context.close();
    // Occlusion still stops, reappearance and a fresh key do NOT resume.
    sim=await active(browser);page=sim.page;await setpoint(page,.4,.1,25);await delay(300);
    await setpoint(page,.4,.1,25,false);await delay(550);s=await state(page);
    assert(!s.session_requested&&s.trial_ended,'occlusion revokes');
    assert(s.state!=='ACTIVE'&&s.gate.output_zero,'occlusion zero');
    await setpoint(page,.4,.1,25,true);await page.keyboard.press('Space');await delay(200);
    assert(!(await state(page)).session_requested,'occlusion does not auto resume');
    await page.keyboard.press('Escape');r=await result(sim.run);
    results.push({case:'actual_ros_occlusion_no_resume',...r});await sim.context.close();
    // Active browser loss revokes; bound with server original challenge time.
    sim=await active(browser);page=sim.page;await setpoint(page,.5,.1,25);await delay(200);
    await page.evaluate(()=>{const end=performance.now()+265;while(performance.now()<end){}});
    r=await result(sim.run);assert(r.authority_ever_acquired&&r.fault&&!r.hardware_socket_opened);
    assert(r.software_stop_evidence.gate_zero_observed&&!r.software_stop_evidence.held);
    results.push({case:'actual_ros_active_ui_stall',...r});await sim.context.close();
    // Sustained motion/reversals, no incidental renewal by a GUI/X11 worker.
    sim=await active(browser,120);page=sim.page;
    const until=Date.now()+Number(process.env.CONTINUOUS_SECONDS||8)*1000;
    let n=0;while(Date.now()<until){
      const t=n++*.15;await setpoint(page,.2+.20*Math.sin(t),.1+.15*Math.sin(t*.8),20+12*Math.sin(t));
      await delay(150);s=await state(page);assert(s.state==='ACTIVE'&&s.session_requested,JSON.stringify(s));
    }
    await page.keyboard.press('Escape');r=await result(sim.run);
    assert(r.software_stop_evidence.gate_zero_observed&&!r.software_stop_evidence.held);
    results.push({case:'actual_ros_continuous_follow_esc',...r});await sim.context.close();
    console.log('BROWSER_TEST_RESULT '+JSON.stringify({passed:true,cases:results}));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
