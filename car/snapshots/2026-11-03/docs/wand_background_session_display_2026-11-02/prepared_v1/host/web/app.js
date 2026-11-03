'use strict';
const $=id=>document.getElementById(id),token=location.hash.slice(1);
let ws=null,viewWs=null,latest=null,mode='rehearsal',ended=false,resultSeen=false,launcherUrl=null;
let sequence=0,lastView=null,lastViewSequence=0,lastDraw=0,resourcesReady=false,pageReady=false;
let offer=null,keyDown=false,pendingClick=null,startSent=false;
const pageWarmup=new PageWarmup();
function text(id,value){if($(id).textContent!==value)$(id).textContent=value;
 if(id==='connection')$(id).hidden=!/缺少|停止|终止|保护|异常|断开/.test(value);
}
function stop(reason){
 if(ended)return;ended=true;pendingClick=null;keyDown=false;
 if(ws?.readyState===WebSocket.OPEN)ws.send(JSON.stringify({type:'stop',reason}));
 text('instruction','已请求停车：'+reason+'；等待停车确认，不会自动恢复。');
 if(launcherUrl)$('restart').hidden=false;
}
function foreground(){return !document.hidden&&document.hasFocus();}
function freshStart(){return foreground()&&lastView!==null&&performance.now()-lastView<=500
 &&latest?.prepare_phase==='READY'&&offer&&performance.now()-offer.received<=500;}
function connectView(owner){
 viewWs=new WebSocket(`ws://${location.host}/view`);
 viewWs.onopen=()=>viewWs.send(JSON.stringify({type:'view_hello',token,owner}));
 viewWs.onmessage=event=>{
  let v;try{v=JSON.parse(event.data);}catch{stop('VIEW_DATA_INVALID');return;}
  if(v.kind==='view_ready')return;
  if(v.kind!=='view'||!Number.isSafeInteger(v.view_sequence)||v.view_sequence<=lastViewSequence){stop('VIEW_SEQUENCE');return;}
  lastViewSequence=v.view_sequence;lastView=performance.now();
  if(resultSeen){viewWs.send(JSON.stringify({type:'ack',sequence:v.view_sequence}));return;}
  latest=v;
  const n=v.intent_offer;
  offer=n?.protocol==='WAND_BACKEND_SESSION_V1'&&n.ttl_ms===1000&&typeof n.nonce==='string'
    ?{nonce:n.nonce,received:lastView}:null;
  // Render credit only; its absence MUST NOT withdraw backend-session authority.
  viewWs.send(JSON.stringify({type:'ack',sequence:v.view_sequence}));
  if(v.limits)text('mode',`档${v.stage} · 1:1平移＋旋转 · ${v.limits.speed.toFixed(2)}m/s · ${v.limits.accel.toFixed(2)}m/s² · ${v.limits.yaw.toFixed(2)}rad/s · ${v.limits.yaw_accel.toFixed(2)}rad/s²`);
  if(v.channel?.fault&&!ended){ended=true;text('instruction','本轮结束：'+v.channel.fault);if(launcherUrl)$('restart').hidden=false;}
 };
 viewWs.onclose=()=>{if(!resultSeen)stop('VIEW_DISCONNECTED');};
 viewWs.onerror=()=>{if(!resultSeen)stop('VIEW_CONNECTION_ERROR');};
}
function connect(){
 if(!resourcesReady||!pageReady||ws||ended||!foreground())return;
 if(!token){text('connection','缺少令牌，请使用启动器给出的新地址');return;}
 ws=new WebSocket(`ws://${location.host}/control`);
 ws.onopen=()=>ws.send(JSON.stringify({type:'hello',token}));
 ws.onmessage=event=>{
  let v;try{v=JSON.parse(event.data);}catch{stop('CONTROL_DATA_INVALID');return;}
  if(v.kind==='hello'){
   mode=v.mode;
   if(v.authority_mode!=='WAND_BACKEND_SESSION_V1'){stop('AUTHORITY_MODE_MISMATCH');return;}
   if(v.launcher_url){try{const u=new URL(v.launcher_url);if(u.protocol==='http:'&&u.hostname==='127.0.0.1'&&u.port&&u.hash)launcherUrl=u.href;}catch{}}
   connectView(v.owner);$('estop').hidden=false;$('simulation').hidden=true;
   text('stop','停止 / 结束本轮跟随');
   text('footer','READY后单击空格开始；切换窗口、最小化继续；回到网页再次空格 / Esc停止。');
  }else if(v.kind==='denied')stop('DENIED_'+v.reason);
  else if(v.kind==='result'){
   ended=resultSeen=true;const r=v.result,s=r.stop_evidence;
   latest={...latest,goal:null,state:s?.stop_confirmed&&s?.gate_output_zero?'STOPPED_CONFIRMED':'STOP_UNCONFIRMED',reason:r.reason};
   text('instruction',`${latest.state==='STOPPED_CONFIRMED'?'本轮已结束并停稳':'停车尚未确认'}：${r.reason}；不会自动恢复。`);
   if(launcherUrl)$('restart').hidden=false;
  }
 };
 ws.onclose=()=>{if(!resultSeen)stop('CONTROL_DISCONNECTED');};
 ws.onerror=()=>{if(!resultSeen)stop('CONTROL_CONNECTION_ERROR');};
}
Promise.resolve(document.fonts?.ready).then(()=>{resourcesReady=true;connect();});
function cancelPendingClick(){keyDown=false;pendingClick=null;}
// Focus/visibility is a property of the VIEW only, not a motion lease.
window.addEventListener('blur',cancelPendingClick);
document.addEventListener('visibilitychange',cancelPendingClick);
window.addEventListener('focus',()=>{cancelPendingClick();connect();});
window.addEventListener('pagehide',()=>{
 stop('PAGE_CLOSED');
 try{fetch('/operator-stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({type:'stop',token,reason:'PAGE_CLOSED'}),keepalive:true}).catch(()=>{});}catch{}
});
document.addEventListener('keydown',e=>{
 if(!e.isTrusted)return;
 if(e.code==='Escape'){e.preventDefault();stop('ESC');return;}
 if(e.code!=='Space')return;e.preventDefault();
 if(e.repeat||keyDown||ended||ws?.readyState!==WebSocket.OPEN)return;
 keyDown=true;
 if(startSent||latest?.session_requested||latest?.state==='ACTIVE'){stop('SECOND_SPACE');return;}
 if(freshStart())pendingClick={nonce:offer.nonce,at:performance.now()};
},true);
document.addEventListener('keyup',e=>{
 if(e.code!=='Space'||!e.isTrusted)return;e.preventDefault();
 const p=pendingClick,was=keyDown;cancelPendingClick();
 if(!was||!p||ended||!freshStart()||performance.now()-p.at>750)return;
 ws.send(JSON.stringify({type:'command',action:'start',nonce:p.nonce,sequence:++sequence}));startSent=true;
},true);
$('stop').onclick=()=>stop('STOP_BUTTON');$('estop').onclick=()=>stop('ESTOP');
$('restart').onclick=()=>{if(ended&&launcherUrl)location.replace(launcherUrl);};
// No periodic socket sender, UI heartbeat or foreground assertion exists here.
// The existing world-grid renderer (view only) is appended during preparation.

var TrackerWorldView = (() => {
  const finitePose = p => p && Number.isFinite(p.x) && Number.isFinite(p.y);
  function niceStep(value) {
    const power = 10 ** Math.floor(Math.log10(Math.max(value, .01)));
    return [1, 2, 5, 10].find(n => n * power >= value) * power;
  }
  function layout(width, height, car, goal, trace, camera) {
    const usableX = Math.max(40, width / 2 - 48), usableY = Math.max(40, height / 2 - 40);
    const aspect = usableX / usableY;
    let halfY = Math.max(3, camera.halfY || 3);
    const points = (trace || []).filter(p => Array.isArray(p) && p.length >= 2 && p.slice(0, 2).every(Number.isFinite)).slice(-100);
    for (const p of [car, goal]) if (finitePose(p)) points.push([p.x, p.y]);
    for (const p of points) halfY = Math.max(halfY, Math.abs(p[1]) / .84, Math.abs(p[0]) / (aspect * .84));
    // Zoom out only when necessary. No tracking-centred panning or auto zoom-in.
    camera.halfY = Math.ceil(halfY * 2) / 2;
    const scale = usableY / camera.halfY, major = Math.max(1, niceStep(38 / scale));
    return {width, height, scale, major, minor: major / 5,
      origin: [width / 2, height / 2],
      project: (x, y) => [width / 2 + x * scale, height / 2 - y * scale]};
  }
  function draw(ctx, width, height, car, goal, trace, camera) {
    const v = layout(width, height, car, goal, trace, camera), s = v.scale;
    const [ox, oy] = v.origin;
    function background(ctx){
    ctx.fillStyle = '#242b32'; ctx.fillRect(0, 0, width, height);
    const xMin = -ox / s, xMax = (width - ox) / s, yMin = -(height - oy) / s, yMax = oy / s;
    function grid(step, color, lineWidth) {
      ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.beginPath();
      let n = 0;
      for (let x = Math.ceil(xMin / step) * step; x <= xMax && n++ < 180; x += step) {
        const p = v.project(x, 0)[0]; ctx.moveTo(p, 0); ctx.lineTo(p, height);
      }
      n = 0;
      for (let y = Math.ceil(yMin / step) * step; y <= yMax && n++ < 180; y += step) {
        const p = v.project(0, y)[1]; ctx.moveTo(0, p); ctx.lineTo(width, p);
      }
      ctx.stroke();
    }
    grid(v.minor, '#303a44', .7); grid(v.major, '#586573', 1);
    function axis(color, a, b, left, right) {
      ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.beginPath();
      ctx.moveTo(...a); ctx.lineTo(...b); ctx.moveTo(...left); ctx.lineTo(...b); ctx.lineTo(...right); ctx.stroke();
    }
    axis('#d98585', [0, oy], [width - 13, oy], [width - 23, oy - 5], [width - 23, oy + 5]);
    axis('#81c7a4', [ox, height], [ox, 13], [ox - 5, 23], [ox + 5, 23]);
    ctx.font = '13px "Microsoft YaHei",sans-serif'; ctx.fillStyle = '#b7c2cc'; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    let count = 0;
    const label = x => Math.abs(x) < 1e-8 ? '0' : Number(x.toPrecision(6)).toString();
    for (let x = Math.ceil(xMin / v.major) * v.major; x < xMax && count++ < 100; x += v.major) {
      const px = v.project(x, 0)[0];
      if (Math.abs(x) > 1e-7 && px > 24 && px < width - 45) ctx.fillText(label(x), px, oy + 8);
    }
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; count = 0;
    for (let y = Math.ceil(yMin / v.major) * v.major; y < yMax && count++ < 100; y += v.major) {
      const py = v.project(0, y)[1];
      if (Math.abs(y) > 1e-7 && py > 40 && py < height - 25) ctx.fillText(label(y), ox - 9, py);
    }
    ctx.font = 'bold 15px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'right';
    ctx.fillStyle = '#eea1a1'; ctx.fillText('+X', width - 12, oy - 15);
    ctx.fillStyle = '#9bdcba'; ctx.textAlign = 'left'; ctx.fillText('+Y', ox + 12, 20);
    ctx.strokeStyle = '#f0f5fa'; ctx.lineWidth = 1.8; ctx.beginPath();
    ctx.moveTo(ox - 6, oy); ctx.lineTo(ox + 6, oy); ctx.moveTo(ox, oy - 6); ctx.lineTo(ox, oy + 6); ctx.stroke();
    ctx.fillStyle = '#e6edf4'; ctx.font = '13px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'left'; ctx.fillText('0, 0', ox + 9, oy - 13);
    ctx.fillStyle = '#b9c8d6'; ctx.font = 'bold 14px "Microsoft YaHei",sans-serif'; ctx.fillText('VICON WORLD · XY', 15, 21);
    ctx.font = '12px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'right';
    ctx.fillText(`主格 ${label(v.major)} m · 细格 ${label(v.minor)} m`, width - 14, height - 15);
    }
    const key=[width,height,s,v.major].join(':');
    if(typeof document!=='undefined'&&document.createElement){
      if(camera.backgroundKey!==key){
        const layer=document.createElement('canvas');layer.width=width;layer.height=height;
        background(layer.getContext('2d'));camera.background=layer;camera.backgroundKey=key;
      }
      ctx.drawImage(camera.background,0,0);
    }else background(ctx);
    ctx.save(); ctx.beginPath(); ctx.rect(0, 0, width, height); ctx.clip();
    ctx.strokeStyle = '#6aabd0'; ctx.lineWidth = 1.7; ctx.beginPath();
    let begun = false;
    for (const p of (trace || []).slice(-100)) {
      if (!Array.isArray(p) || p.length < 2 || !p.slice(0, 2).every(Number.isFinite)) continue;
      const xy = v.project(p[0], p[1]); if (begun) ctx.lineTo(...xy); else {ctx.moveTo(...xy); begun = true;}
    }
    ctx.stroke();
    function body(p, isCar) {
      if (!finitePose(p) || !Number.isFinite(p.yaw)) return;
      const [x, y] = v.project(p.x, p.y), size = Math.max(18, Math.min(38, .21 * s));
      const color = isCar ? '#59c2f5' : '#f5b44e';
      ctx.save(); ctx.translate(x, y); ctx.rotate(-p.yaw); ctx.strokeStyle = color; ctx.lineWidth = 2;
      if (isCar) {
        ctx.fillStyle = '#245a7777'; ctx.fillRect(-size / 2, -size / 2, size, size);
        ctx.strokeRect(-size / 2, -size / 2, size, size);
        ctx.beginPath(); ctx.moveTo(-size / 2, -size / 2); ctx.lineTo(size / 2, size / 2);
        ctx.moveTo(-size / 2, size / 2); ctx.lineTo(size / 2, -size / 2); ctx.stroke();
      } else {
        // The same mapped target as before, depicted as a T rather than a dot.
        ctx.beginPath(); ctx.moveTo(-size * .6, 0); ctx.lineTo(size * .45, 0);
        ctx.moveTo(size * .45, -size * .5); ctx.lineTo(size * .45, size * .5); ctx.stroke();
      }
      ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(size * .95, 0);
      ctx.lineTo(size * .68, -4); ctx.moveTo(size * .95, 0); ctx.lineTo(size * .68, 4); ctx.stroke();
      ctx.restore(); ctx.fillStyle = color; ctx.font = 'bold 14px "Microsoft YaHei",sans-serif'; ctx.textAlign = 'left'; ctx.textBaseline = isCar ? 'top' : 'bottom';
      const labelY = isCar ? y + size / 2 + 7 : y - size / 2 - 8;
      ctx.fillText(isCar ? 'car01' : 'Sign目标', Math.max(5, Math.min(width - 80, x + 13)), Math.max(42, Math.min(height - 24, labelY)));
    }
    body(goal, false); body(car, true); ctx.restore();
    return v;
  }
  return {layout, draw};
})();
if (typeof module !== 'undefined' && module.exports) module.exports = TrackerWorldView;

const trackerCamera={halfY:3};let fontsPrimed=false,lastText=0;
function render(now){
  requestAnimationFrame(render);if(now-lastDraw<33||(!latest&&pageReady))return;lastDraw=now;
  const drawStarted=performance.now();
  if(!pageReady){
    const canvas=$('map'),box=canvas.getBoundingClientRect();canvas.width=Math.round(box.width);canvas.height=Math.round(box.height);
    if(!resourcesReady||document.hidden||!document.hasFocus()){
      text('instruction','请点击页面；画布预热后连接，尚未使能。');return;
    }
    if(!fontsPrimed){
      // Prime the actual Chinese sprite fonts offscreen too, not just DOM fonts.
      const layer=document.createElement('canvas');layer.width=canvas.width;layer.height=canvas.height;
      TrackerWorldView.draw(layer.getContext('2d'),layer.width,layer.height,{x:0,y:0,yaw:0},{x:1,y:1,yaw:0},[],{});fontsPrimed=true;
    }
    TrackerWorldView.draw(canvas.getContext('2d'),canvas.width,canvas.height,null,null,[],trackerCamera);
    pageReady=pageWarmup.frame(performance.now(),performance.now()-drawStarted);
    text('instruction',pageReady?'页面已就绪，正在连接；尚未使能。':'正在预热画布与字体；尚未使能。');
    if(pageReady)connect();return;
  }
  if(now-lastText>=100){lastText=now;
  if(!resultSeen&&!ended)text('instruction',latest.instruction||'等待4WSC后端数据');
  const c=latest.car;
  text('status',`状态：${latest.state}　原因：${latest.reason||'无'}　模式：${mode==='hardware'?'真实硬件':'回环软件排练'}`);
  if(c)text('pose',`小车 X=${c.x.toFixed(3)}m　Y=${c.y.toFixed(3)}m　朝向=${(c.yaw*180/Math.PI).toFixed(1)}°`);
  text('countdown',`剩余 ${(latest.remaining_s??0).toFixed(1)} 秒`);
  }
  const c=latest.car,g=latest.goal,trace=latest.trace||[];
  const canvas=$('map'),box=canvas.getBoundingClientRect(),w=box.width,h=box.height;
  if(canvas.width!==Math.round(w)||canvas.height!==Math.round(h)){canvas.width=Math.round(w);canvas.height=Math.round(h);}
  const ctx=canvas.getContext('2d');ctx.setTransform(1,0,0,1,0,0);
  TrackerWorldView.draw(ctx,w,h,c,g,trace,trackerCamera);
}
requestAnimationFrame(render);connect();
