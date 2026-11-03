'use strict';
const $=id=>document.getElementById(id),token=location.hash.slice(1),guard=new InputGuard();
let ws=null,viewWs=null,sequence=0,events=[],latest=null,controlStatus={},mode='readonly',ended=false,resultSeen=false;
let lastDraw=0,space=false,clientMaxGap=0,lastView=null,lastViewSequence=0,pairStarted=null;
let replyCount=0,maxReplyDispatch=0,maxSendCall=0;
function text(id,value){if($(id).textContent!==value)$(id).textContent=value;}
function stop(reason){
  if(ended)return;ended=true;events=[];space=false;
  if(ws&&ws.readyState===WebSocket.OPEN)ws.send(JSON.stringify({type:'stop',reason}));
  text('connection','本轮已停止：'+reason+'；不会自动恢复');text('instruction','停止原因：'+reason+'；保持停止，不要在旧窗口重试。');
}
function safeInput(now){
  if(!guard.permit(now)){stop(guard.fault||'INPUT_CLOCK_NOT_READY');return false;}
  if(document.hidden||!document.hasFocus()){stop('WINDOW_NOT_FOREGROUND');return false;}
  if(lastView===null)return false; // No unseen enable request before the first displayed frame.
  if(now-lastView>100){stop('VIEW_STALE_100MS');return false;}
  if(ws.bufferedAmount>4096){stop('CLIENT_BACKPRESSURE');return false;}
  return true;
}
function reply(challenge,entered){
  if(ended||!safeInput(entered))return;
  if(challenge.protocol!=='WAND_BROWSER_V1'||challenge.ttl_ms!==100||typeof challenge.nonce!=='string'){
    stop('CHALLENGE_SCHEMA');return;
  }
  if(events.length>8){stop('CLIENT_EVENT_OVERFLOW');return;}
  const pending=events;events=[];
  // Only this foreground MAIN-thread callback creates the reply; never a Worker,
  // interval sender, view ACK or Node transport. It does NOT advance guard.last.
  const encoded=JSON.stringify({type:'pulse',nonce:challenge.nonce,sequence:++sequence,
    focused:true,visible:true,space,ui_gap_ms:guard.age(entered),events:pending});
  const dispatched=performance.now();
  if(!safeInput(dispatched))return; // Recheck if encoding/OS scheduling consumed the deadline.
  maxReplyDispatch=Math.max(maxReplyDispatch,dispatched-entered);
  ws.send(encoded);maxSendCall=Math.max(maxSendCall,performance.now()-dispatched);
  replyCount++;
}
function connectView(owner){
  if(viewWs)return;pairStarted=performance.now();
  viewWs=new WebSocket(`ws://${location.host}/view`);
  viewWs.onopen=()=>viewWs.send(JSON.stringify({type:'view_hello',token,owner}));
  viewWs.onmessage=event=>{
    let v;try{v=JSON.parse(event.data);}catch{stop('VIEW_DATA_INVALID');return;}
    if(v.kind==='view_ready')return;
    if(v.kind!=='view'||!Number.isSafeInteger(v.view_sequence)||v.view_sequence<=lastViewSequence){stop('VIEW_SEQUENCE');return;}
    const firstView=lastView===null;
    lastViewSequence=v.view_sequence;lastView=performance.now();latest={...v,...controlStatus};
    if(firstView&&!ended)text('connection','双通道已连接；挑战即时回应；严格上限100ms');
    // A view ACK is credit only: it cannot refresh browser proof/lease.
    viewWs.send(JSON.stringify({type:'ack',sequence:v.view_sequence}));
  };
  viewWs.onerror=()=>{if(!resultSeen)stop('VIEW_CONNECTION_ERROR');};
  viewWs.onclose=()=>{if(!resultSeen)stop('VIEW_DISCONNECTED');};
}
let resourcesReady=false;
Promise.resolve(document.fonts?.ready).then(()=>{resourcesReady=true;connect();});
function connect(){
  if(!resourcesReady||ws||ended||document.hidden||!document.hasFocus())return;
  if(!token){text('connection','缺少令牌：请使用终端打印的新完整地址');return;}
  ws=new WebSocket(`ws://${location.host}/control`);
  ws.onopen=()=>{guard.begin(performance.now());ws.send(JSON.stringify({type:'hello',token}));};
  ws.onmessage=event=>{
    const entered=performance.now();let v;
    try{v=JSON.parse(event.data);}catch{stop('CONTROL_DATA_INVALID');return;}
    if(v.kind==='challenge'){reply(v,entered);return;}
    if(v.kind==='hello'){
      mode=v.mode;connectView(v.owner);
      text('mode',mode==='readonly'?'严格只读验收 · 即时回应 · 控制/画面分流 · 无ROS/实车输出':'直接Sign · 1:1平移＋旋转 · 直行1.20 / 斜向0.85m/s · 4.00m/s² · 无实车输出');
      if(v.synthetic)text('mode','自动测试手势 → ROS软件车 · 不是实时Sign · 无实车输出');
      $('simulation').hidden=true;$('estop').hidden=false;
      text('stop',mode==='readonly'?'结束本次只读检查':'停止 / 结束本轮软件跟随');
      text('footer',mode==='readonly'?'严格只读：保持网页前台60秒；空格只记录，不准备、不启动小车。':'软件模式：自动READY后单击空格开始；再次空格/Esc停止；松键不停。失焦、遮挡、断连仍停车。');
      text('connection','双通道连接中；挑战即时回应；严格上限100ms');
    }else if(v.kind==='status'){
      controlStatus=v;latest={...latest,...v};
      if(v.channel?.fault){ended=true;text('connection','保护终止：'+v.channel.fault+'；不会自动恢复');}
    }else if(v.kind==='denied'){stop('DENIED_'+v.reason);}
    else if(v.kind==='result'){
      resultSeen=ended=true;
      const stopped=v.result.software_stop_evidence;
      latest={...latest,result:v.result,session_requested:false,trial_ended:true,
        state:stopped?.stop_confirmed&&stopped?.gate_output_zero?'STOPPED_CONFIRMED':'STOP_UNCONFIRMED',
        reason:v.result.reason,goal:null};
      const reason=v.result.reason;
      text('connection',v.result.passed?'本轮完成并停稳':(reason==='CLIENT_ESC'||reason==='CLIENT_STOP_BUTTON'||reason==='OPERATOR_SECOND_SPACE'?'已手动结束并保存：':'本轮保护结束：')+reason);
      text('instruction',(v.result.passed?'本轮完成':'本轮已结束，原因：'+reason)+'；记录已保存到Linux端，不会自动恢复。');
    }
  };
  ws.onerror=()=>{if(!resultSeen)stop('CONTROL_CONNECTION_ERROR');};
  ws.onclose=()=>{if(!resultSeen)stop('CONTROL_DISCONNECTED');};
}
function periodic(){
  const now=performance.now();if(!ws){connect();return;}
  if(ended||!guard.started)return;
  if(!guard.tick(now)){stop(guard.fault);return;}
  clientMaxGap=guard.maxGap;
  if(document.hidden||!document.hasFocus()){stop('WINDOW_NOT_FOREGROUND');return;}
  if(lastView!==null&&now-lastView>100){stop('VIEW_STALE_100MS');return;}
  if(lastView===null&&pairStarted!==null&&now-pairStarted>2000)stop('VIEW_START_TIMEOUT');
  // No periodic socket sender: only fresh challenge callbacks can reply.
}
setInterval(periodic,20);
window.addEventListener('focus',connect);
window.addEventListener('blur',()=>{if(ws)stop('WINDOW_BLUR');});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&ws)stop('WINDOW_HIDDEN');else connect();});
window.addEventListener('pagehide',()=>stop('PAGE_CLOSED'));
document.addEventListener('keydown',event=>{
  if(!event.isTrusted)return;
  if(event.code==='Escape'){event.preventDefault();stop('ESC');return;}
  if(event.code!=='Space')return;event.preventDefault();
  if(event.repeat||space||ended||!ws||ws.readyState!==WebSocket.OPEN)return;
  space=true;events.push({kind:'press'});
},true);
document.addEventListener('keyup',event=>{
  if(event.code!=='Space'||!event.isTrusted)return;event.preventDefault();
  if(space&&!ended)events.push({kind:'release'});space=false;
},true);
$('stop').onclick=()=>stop('STOP_BUTTON');$('estop').onclick=()=>stop('ESTOP');
function setpoint(){
  if(mode!=='software'||ended||!ws||ws.readyState!==WebSocket.OPEN)return;
  events=events.filter(v=>v.kind!=='setpoint');
  events.push({kind:'setpoint',x:Number($('x').value),y:Number($('y').value),yaw:Number($('yaw').value)*Math.PI/180,visible:$('visible').checked});
}
for(const id of ['x','y','yaw','visible'])$(id).addEventListener('input',setpoint);
$('center').onclick=()=>{for(const id of ['x','y','yaw'])$(id).value='0';setpoint();};
// The existing local renderer is appended by the offline preparation script.

function render(now){
  requestAnimationFrame(render);if(now-lastDraw<33||!latest)return;lastDraw=now;
  if(!resultSeen)text('instruction',latest.instruction||'等待软件后端数据');
  const c=latest.car,g=latest.goal,trace=latest.trace||[];
  text('status',`状态：${latest.state}　原因：${latest.reason||'无'}　模式：${mode==='readonly'?'只读软件样例':'ROS软件仿真'}`);
  text('timing',`UI周期最大 ${guard.maxGap.toFixed(1)}ms；回应调度最大 ${maxReplyDispatch.toFixed(1)}ms；严格100ms`);
  if(c)text('pose',`软件车 X=${c.x.toFixed(3)}m　Y=${c.y.toFixed(3)}m　朝向=${(c.yaw*180/Math.PI).toFixed(1)}°`);
  text('countdown',`剩余 ${(latest.remaining_s??0).toFixed(1)} 秒`);
  const canvas=$('map'),box=canvas.getBoundingClientRect(),dpr=1,w=box.width,h=box.height;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const points=[...trace];for(const p of [c,g])if(p)points.push([p.x,p.y]);if(!points.length)return;
  let xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);const cx=(Math.max(...xs)+Math.min(...xs))/2,cy=(Math.max(...ys)+Math.min(...ys))/2;
  const hx=Math.max(1,(Math.max(...xs)-Math.min(...xs))/2+.3),hy=Math.max(1,(Math.max(...ys)-Math.min(...ys))/2+.3),scale=Math.min((w-50)/(2*hx),(h-55)/(2*hy));
  const point=(x,y)=>[w/2+(x-cx)*scale,h/2-(y-cy)*scale];
  ctx.strokeStyle='#93bad6';ctx.lineWidth=2;ctx.beginPath();trace.forEach((p,i)=>{const xy=point(...p);i?ctx.lineTo(...xy):ctx.moveTo(...xy);});ctx.stroke();
  for(const [p,color] of [[g,'#d38d1f']])if(p){const [x,y]=point(p.x,p.y);ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,7,0,2*Math.PI);ctx.stroke();ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(...point(p.x+.23*Math.cos(p.yaw),p.y+.23*Math.sin(p.yaw)));ctx.stroke();}
  if(c){const [x,y]=point(c.x,c.y);ctx.save();ctx.translate(x,y);ctx.rotate(-c.yaw);ctx.fillStyle='#2986c1';ctx.fillRect(-.095*scale,-.105*scale,.19*scale,.21*scale);ctx.strokeStyle='white';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(.09*scale,0);ctx.stroke();ctx.restore();}
  ctx.fillStyle='#a23925';ctx.font='18px Microsoft YaHei,sans-serif';ctx.fillText('视图不是围栏；Vicon可见 ≠ 周围无障碍',12,26);
}
requestAnimationFrame(render);connect();
