'use strict';
// Actual page in headless Chrome with NO real network, ROS, or motor endpoint.
// Socket callbacks are supplied deterministic display fixtures. Browser clock
// guards are tested separately by the retained 24 DOM/VM regressions.
const fs=require('fs'),path=require('path'),assert=require('assert');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(process.argv[2]),out=path.resolve(process.argv[3]);
fs.mkdirSync(out,{recursive:true});
let checks=0;
function check(value,message){assert(value,message);checks++;}
const removed=['最长600秒受控试运行','停车/电流/温升未量化','无中间参考点','无电子围栏',
  '视图不是围栏','连接与安全状态','双通道已连接','UI周期最大','失焦、遮挡、断连仍停车','松键不停'];
async function scene(context,width,height){
  const page=await context.newPage();await page.setViewportSize({width,height});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.addInitScript(()=>{
    // Never instantiate the browser's network-capable WebSocket.
    window.__displaySends=[];window.__displaySockets=[];
    window.WebSocket=class {
      static OPEN=1;
      constructor(url){this.url=url;this.readyState=1;this.bufferedAmount=0;window.__displaySockets.push(this);}
      send(raw){window.__displaySends.push(JSON.parse(raw));}
      close(){this.readyState=3;}
    };
    // This is a render fixture, not an operator-attention or timing acceptance.
    document.hasFocus=()=>true;
  });
  await page.goto('http://demo-ui.invalid/#render-fixture-only');
  await page.waitForFunction(()=>window.__displaySockets.length>0);
  await page.evaluate(()=>{
    const control=window.__displaySockets[0];
    control.onmessage({data:JSON.stringify({kind:'hello',mode:'hardware',owner:'DISPLAY_FIXTURE',purpose:'sign'})});
    control.onmessage({data:JSON.stringify({kind:'status',stage:6,purpose:'sign',state:'ACTIVE',reason:'',
      limits:{speed:.70,accel:2.,yaw:2.,yaw_accel:12.},remaining_s:527.2,
      instruction:'单击跟随中，剩余≤527.2秒；松键继续；再次空格、Esc或停止按钮结束'})});
    const view=window.__displaySockets.find(x=>x.url.endsWith('/view'));
    view.onmessage({data:JSON.stringify({kind:'view',view_sequence:1,state:'ACTIVE',remaining_s:527.2,
      car:{x:.436,y:-1.482,yaw:2.578},goal:{x:.52,y:-1.4,yaw:2.7},trace:[[0,-1],[.2,-1.2],[.436,-1.482]]})});
  });
  await page.waitForFunction(()=>document.getElementById('pose').textContent.includes('0.436'));
  return {page,errors};
}
(async()=>{
  const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--disable-gpu']});
  try{
    const context=await browser.newContext();
    await context.route('**/*',async route=>{
      const url=new URL(route.request().url());
      assert.equal(url.hostname,'demo-ui.invalid','all other requests forbidden');
      if(url.pathname==='/diagnostic'){await route.fulfill({status:204});return;}
      const name=url.pathname==='/'?'index.html':url.pathname.slice(1);
      assert(['index.html','app.js','input_guard.js','focus_evidence.js','style.css'].includes(name),'only fixed static UI files');
      await route.fulfill({status:200,body:fs.readFileSync(path.join(root,'web',name)),
        contentType:name.endsWith('.html')?'text/html; charset=utf-8':name.endsWith('.css')?'text/css; charset=utf-8':'text/javascript; charset=utf-8'});
    });
    for(const [width,height] of [[1280,720],[1920,1080]]){
      const {page,errors}=await scene(context,width,height);
      const visible=await page.locator('body').innerText();
      for(const phrase of removed)check(!visible.includes(phrase),'marked text absent: '+phrase);
      check(!(await page.locator('#connection').isVisible()),'healthy connection diagnostics hidden');
      check(!(await page.locator('#timing').isVisible()),'healthy scheduler diagnostics hidden');
      check(await page.locator('#stop').isVisible(),'stop button visible');
      check(await page.locator('#estop').isVisible(),'emergency stop visible');
      check(visible.includes('527.2'),'countdown retained');
      check(visible.includes('0.70m/s')&&visible.includes('2.00m/s²'),'real parameters retained');
      check(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'no horizontal overflow');
      const before=await page.evaluate(()=>window.__displaySends.filter(x=>x.type==='pulse').length);
      check(before===0,'render fixture acquired no authority');
      await page.screenshot({path:path.join(out,`demo_${width}x${height}.png`),fullPage:true});
      await page.keyboard.press('Escape');
      check(await page.evaluate(()=>window.__displaySends.some(x=>x.type==='stop'&&x.reason==='ESC')),'trusted Esc still sends stop');
      check(await page.locator('#connection').isVisible(),'fault/stop message remains visible');
      check((await page.locator('#connection').innerText()).includes('ESC'),'stop reason retained');
      check(errors.length===0,'no page JavaScript errors');
      await page.close();
    }
    for(const [id,reason] of [['stop','STOP_BUTTON'],['estop','ESTOP']]){
      const {page}=await scene(context,1280,720);await page.locator('#'+id).click();
      check(await page.evaluate(r=>window.__displaySends.some(x=>x.type==='stop'&&x.reason===r),reason),id+' still sends stop');
      await page.close();
    }
    const result={passed:true,checks,actual_headless_browser:true,render_fixture_only:true,
      timing_acceptance:false,real_network_requests:0,hardware_socket_opened:false,
      ros_initialized:false,physical_motion_started:false,all_stop_controls_preserved:true};
    fs.writeFileSync(path.join(out,'RESULT.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
    await context.close();
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
