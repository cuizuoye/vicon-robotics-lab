'use strict';
const {chromium}=require('/home/PRIMARY_USER/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const {spawn}=require('child_process'),path=require('path'),fs=require('fs'),assert=require('assert');
const root=process.env.FAST_OUTPUT||'/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_fast_2026-10-28/prepared';
const wait=ms=>new Promise(r=>setTimeout(r,ms));
async function launch(browser){
  const p=spawn(process.execPath,[path.join(root,'server.cjs'),'--readonly','--port','0','--seconds','10']);let out='';
  p.stdout.on('data',b=>out+=b);p.stderr.on('data',b=>process.stderr.write(b));
  for(let n=0;n<500&&!out.includes('BROWSER_READY ');n++)await wait(10);
  assert(out.includes('BROWSER_READY '));const info=JSON.parse(out.split('BROWSER_READY ')[1].split('\n')[0]);
  const context=await browser.newContext({viewport:{width:1280,height:720}}),page=await context.newPage();
  await page.goto(info.url);await page.click('h1');await page.waitForFunction(()=>replyCount>5);
  return {p,...info,context,page};
}
async function result(run){
  for(let n=0;n<500&&!fs.existsSync(path.join(run.session,'result.json'));n++)await wait(10);
  const r=JSON.parse(fs.readFileSync(path.join(run.session,'result.json')));
  assert(!r.ros_initialized&&!r.authority_ever_acquired&&!r.hardware_socket_opened&&!r.physical_motion_started);
  return r;
}
(async()=>{
  const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']}),cases=[];
  try{
    let run=await launch(browser);await run.page.evaluate(()=>{const end=performance.now()+265;while(performance.now()<end){}});
    let r=await result(run);assert(!r.passed&&r.fault);cases.push({case:'actual_browser265ms_stall_revokes',reason:r.reason});await run.context.close();
    run=await launch(browser);await run.page.evaluate(()=>window.dispatchEvent(new Event('blur')));
    r=await result(run);assert(!r.passed&&r.reason.includes('WINDOW'));cases.push({case:'focus_loss_revokes',reason:r.reason});await run.context.close();
    run=await launch(browser);await run.page.evaluate(()=>viewWs.close());
    r=await result(run);assert(!r.passed&&r.reason.includes('VIEW'));cases.push({case:'view_loss_revokes_with_control_alive',reason:r.reason});await run.context.close();
    run=await launch(browser);await run.page.keyboard.press('Escape');r=await result(run);
    assert(r.reason==='CLIENT_ESC');cases.push({case:'Esc_terminal_no_resume',reason:r.reason});await run.context.close();
    console.log('BROWSER_SAFETY_TEST '+JSON.stringify({passed:true,cases}));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
