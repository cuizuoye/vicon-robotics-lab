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
  lastViewSequence=v.view_sequence;lastView=performance.now();latest=v;
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
