'use strict';
const {chromium}=require('/home/PRIMARY_USER/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const {spawn}=require('child_process'),path=require('path'),fs=require('fs'),assert=require('assert'),net=require('net');
const root=process.env.PROBE_OUTPUT||'/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_probe_2026-10-27/prepared';
const wait=ms=>new Promise(r=>setTimeout(r,ms));
async function launch(seconds){
  const child=spawn(process.execPath,[path.join(root,'server.cjs'),'--port','0','--seconds',String(seconds)]);
  let out='',err='';child.stdout.on('data',b=>out+=b);child.stderr.on('data',b=>err+=b);
  for(let n=0;n<500&&!out.includes('BROWSER_READY ');n++){if(child.exitCode!==null)throw Error(out+err);await wait(10);}
  assert(out.includes('BROWSER_READY '));return {child,...JSON.parse(out.split('BROWSER_READY ')[1].split('\n')[0])};
}
async function result(run){
  for(let n=0;n<500&&!fs.existsSync(path.join(run.session,'result.json'));n++)await wait(10);
  return JSON.parse(fs.readFileSync(path.join(run.session,'result.json')));
}
(async()=>{
  const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
  const cases=[];
  try{
    let run=await launch(Number(process.env.PROBE_SECONDS||60)),page=await browser.newPage({viewport:{width:1280,height:720}});
    await page.goto(run.url);await page.click('h1');
    await page.waitForFunction(()=>latest?.diagnostics?.raw_replies>=3);
    await page.waitForFunction(()=>resultSeen,{},{timeout:(Number(process.env.PROBE_SECONDS||60)+5)*1000});
    let r=await result(run);assert(r.passed&&r.diagnostic_completed&&!r.ros_initialized&&!r.authority_ever_acquired);
    assert(r.timings.reader_to_main_s&&r.timings.issue_to_node_receive_s,'host monotonic stage stamps valid');
    cases.push({case:'actual_browser_clean_readonly',...r});await page.close();
    run=await launch(3);page=await browser.newPage({viewport:{width:1280,height:720}});
    await page.goto(run.url);await page.click('h1');await page.waitForFunction(()=>latest?.diagnostics?.raw_replies>=3);
    // MAIN-thread pause must fail strict validation but NOT hide subsequent samples/animation.
    await page.evaluate(()=>{const end=performance.now()+265;while(performance.now()<end){}});
    await page.waitForFunction(()=>latest?.diagnostics?.post_failure_samples>=4);
    const before=await page.evaluate(()=>latest.car.x);await wait(150);
    assert(Math.abs((await page.evaluate(()=>latest.car.x))-before)>.02,'animation continues ONLY as observation');
    await page.screenshot({path:path.join(run.session,'failed_strict_but_observing.png')});
    await page.waitForFunction(()=>resultSeen);
    r=await result(run);assert(!r.passed&&r.diagnostic_completed&&r.post_failure_samples>10&&r.max_client_gap_s>.2);
    assert(!r.authority_ever_acquired&&r.strict_fault);cases.push({case:'main_thread_pause_latched_observer_continues',...r});await page.close();
    // Real TCP forwarder, delayed server->browser packets; resembles a batching tunnel, not a real Windows test.
    run=await launch(4);let delayed=false;
    const relay=net.createServer(local=>{
      const remote=net.connect(run.port,'127.0.0.1');let buffer=[];
      local.setNoDelay(true);remote.setNoDelay(true);local.pipe(remote);
      remote.on('data',b=>{if(delayed)buffer.push(b);else local.write(b);});
      const pump=setInterval(()=>{if(buffer.length){local.write(Buffer.concat(buffer));buffer=[];}},180);
      local.on('close',()=>{clearInterval(pump);remote.destroy();});remote.on('close',()=>local.destroy());
      local.on('error',()=>{});remote.on('error',()=>{});
    });
    await new Promise(r=>relay.listen(0,'127.0.0.1',r));const port=relay.address().port;
    // Test with a WS client behind the raw relay, preserving the same browser pulse schema.
    const WS=require('/usr/share/nodejs/ws');
    const ws=new WS(`ws://127.0.0.1:${port}/control`,{origin:`http://127.0.0.1:${run.port}`,headers:{Host:`127.0.0.1:${run.port}`}});
    let seq=0,nonce=null;ws.on('error',()=>{});
    ws.on('open',()=>ws.send(JSON.stringify({type:'hello',token:run.url.split('#')[1]})));
    ws.on('message',b=>{const m=JSON.parse(b);if(m.kind==='challenge'){nonce=m.nonce;ws.send(JSON.stringify({type:'diagnostic_echo',nonce:m.nonce}));}});
    const pulse=setInterval(()=>{if(nonce&&ws.readyState===WS.OPEN){const use=nonce;nonce=null;
      ws.send(JSON.stringify({type:'pulse',nonce:use,sequence:++seq,focused:true,visible:true,space:false,ui_gap_ms:20,events:[]}));}},20);
    await wait(150);delayed=true;
    r=await result(run);clearInterval(pulse);ws.terminate();await new Promise(r=>relay.close(r));
    assert(r.diagnostic_completed&&!r.passed&&r.timings.echo_roundtrip_s.max>.15&&r.post_failure_samples>5);
    assert(r.max_client_gap_s===.02&&!r.ros_initialized&&!r.authority_ever_acquired);
    cases.push({case:'tcp180ms_batching_not_browser_stall',...r});
    console.log('PROBE_BROWSER_TEST '+JSON.stringify({passed:true,cases}));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
