'use strict';
const {chromium}=require('/home/PRIMARY_USER/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const {spawn}=require('child_process'),path=require('path'),fs=require('fs'),assert=require('assert');
const root=process.env.FAST_OUTPUT||'/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_fast_2026-10-28/prepared';
const wait=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
  const child=spawn(process.execPath,[path.join(root,'server.cjs'),'--readonly','--port','0','--seconds','60']);
  let out='',err='';child.stdout.on('data',b=>out+=b);child.stderr.on('data',b=>err+=b);
  for(let n=0;n<500&&!out.includes('BROWSER_READY ');n++)await wait(10);
  assert(out.includes('BROWSER_READY '),err);const run=JSON.parse(out.split('BROWSER_READY ')[1].split('\n')[0]);
  const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
  try{
    const page=await browser.newPage({viewport:{width:1280,height:720}}),errors=[];page.on('pageerror',e=>errors.push(String(e)));
    await page.goto(run.url);await page.click('h1');await page.waitForFunction(()=>replyCount>5);
    await page.keyboard.press('Space');await page.screenshot({path:path.join(run.session,'fast_readonly_1280x720.png')});
    await page.waitForFunction(()=>resultSeen,{},{timeout:65000});
    const client=await page.evaluate(()=>({replyCount,maxReplyDispatch,maxSendCall,periodicMax:guard.maxGap,lastViewSequence}));
    const r=JSON.parse(fs.readFileSync(path.join(run.session,'result.json')));
    for(let n=0;n<50&&!fs.existsSync(path.join(run.session,'transport.json'));n++)await wait(10);
    const transport=JSON.parse(fs.readFileSync(path.join(run.session,'transport.json')));
    const observation={session:run.session,client,transport,proof_max_age_s:r.max_proof_age_s,elapsed_s:r.elapsed_s,
      performance_target_10ms_met:client.maxReplyDispatch<10,acceptance_limit_ms:100,
      metric_note:'Dispatch-before-send is distinct from WebSocket.send call cost;10ms is an observational target, NOT a safety limit.'};
    fs.writeFileSync(path.join(run.session,'browser_metrics.json'),JSON.stringify(observation,null,2)+'\n');
    assert(r.passed&&r.completed&&!r.ros_initialized&&!r.authority_ever_acquired&&!r.hardware_socket_opened);
    assert(r.key_events.press===1&&r.key_events.release===1&&errors.length===0);
    assert(Number.isFinite(client.maxReplyDispatch)&&client.maxReplyDispatch<100&&client.periodicMax<100&&client.replyCount>3000);
    assert(transport.view.max_pending===1&&transport.view.max_frame_bytes<=8192&&transport.max_control_buffer<=8192);
    console.log('METRICS_TEST '+JSON.stringify({passed:true,...observation}));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
