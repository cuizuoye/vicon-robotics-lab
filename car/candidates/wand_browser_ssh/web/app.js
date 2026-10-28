'use strict';
const $=id=>document.getElementById(id),token=location.hash.slice(1);
let ws=null,nonce=null,sequence=0,lastPulse=null,events=[],latest=null,mode='readonly',ended=false,resultSeen=false;
let lastDraw=0,space=false,clientMaxGap=0;
function text(id,value){if($(id).textContent!==value)$(id).textContent=value;}
function stop(reason){
  if(ended)return;ended=true;events=[];space=false;
  if(ws&&ws.readyState===WebSocket.OPEN)ws.send(JSON.stringify({type:'stop',reason}));
  text('connection','本轮已停止：'+reason+'；不会自动恢复');
}
function connect(){
  if(ws||ended||document.hidden||!document.hasFocus())return;
  if(!token){text('connection','缺少访问令牌：请使用Linux终端给出的完整地址');return;}
  ws=new WebSocket(`ws://${location.host}/control`);
  ws.onopen=()=>{ws.send(JSON.stringify({type:'hello',token}));lastPulse=performance.now();};
  ws.onmessage=event=>{
    let value;try{value=JSON.parse(event.data);}catch{stop('INVALID_SERVER_DATA');return;}
    if(value.kind==='challenge')nonce=value.nonce;
    else if(value.kind==='hello'){
      mode=value.mode;
      text('mode',mode==='readonly'?'只读验收 · 软件动画 · 不接ROS / Vicon / 小车':'私有ROS软件仿真 · 平移＋旋转1:1 · 无实车输出');
      $('simulation').hidden=mode!=='software';$('estop').hidden=mode!=='software';
      text('stop',mode==='readonly'?'结束本次只读检查':'停止 / 结束本轮软件跟随');
      text('footer',mode==='readonly'?'只读：空格仅记录，不启动小车。请保持页面前台直到自动结束。':'软件模式：READY后单击空格开始；再次空格 / Esc停止；松键不停。失焦、超时或遮挡会停车。');
      text('connection','已连接数据通道；画面在浏览器本地绘制');
    }else if(value.kind==='state'){
      latest=value;if(value.channel?.fault){ended=true;text('connection','保护终止：'+value.channel.fault+'；不会自动恢复');}
    }else if(value.kind==='denied'){ended=true;text('connection','连接被拒绝：'+value.reason);}
    else if(value.kind==='result'){
      resultSeen=ended=true;latest={...latest,result:value.result};
      text('connection',value.result.passed?'本轮验收完成，通过':'本轮未通过：'+value.result.reason);
      text('instruction','结果已自动保存在Linux端，无需复制终端长报文。');
    }
  };
  ws.onerror=()=>text('connection','SSH / WebSocket连接错误；不会自动重连');
  ws.onclose=()=>{if(!resultSeen){ended=true;text('connection','连接已断开：已撤能；不会自动恢复，请查看Linux端保存的结果。');}};
}
function pulse(){
  const now=performance.now();
  if(!ws){connect();return;}
  if(ended||ws.readyState!==WebSocket.OPEN)return;
  const gap=lastPulse===null?0:now-lastPulse;lastPulse=now;clientMaxGap=Math.max(clientMaxGap,gap);
  if(gap>100||gap<0){stop('CLIENT_LOOP_GAP');return;}
  if(document.hidden||!document.hasFocus()){stop('WINDOW_NOT_FOREGROUND');return;}
  if(ws.bufferedAmount>8192){stop('CLIENT_BACKPRESSURE');ws.close();return;}
  if(!nonce)return;
  const use=nonce;nonce=null;const pending=events;events=[];
  ws.send(JSON.stringify({type:'pulse',nonce:use,sequence:++sequence,focused:true,visible:true,space,
    ui_gap_ms:gap,events:pending}));
}
setInterval(pulse,20);
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
$('stop').onclick=()=>stop('STOP_BUTTON');
$('estop').onclick=()=>{if(!ended){events.push({kind:'estop'});pulse();}};
function setpoint(){
  if(mode!=='software'||ended||!ws||ws.readyState!==WebSocket.OPEN)return;
  events=events.filter(v=>v.kind!=='setpoint');
  events.push({kind:'setpoint',x:Number($('x').value),y:Number($('y').value),yaw:Number($('yaw').value)*Math.PI/180,visible:$('visible').checked});
}
for(const id of ['x','y','yaw','visible'])$(id).addEventListener('input',setpoint);
$('center').onclick=()=>{for(const id of ['x','y','yaw'])$(id).value='0';setpoint();};
function render(now){
  requestAnimationFrame(render);if(now-lastDraw<45||!latest)return;lastDraw=now;
  if(!resultSeen)text('instruction',latest.instruction||'等待软件后端数据');
  const c=latest.car,g=latest.goal,r=latest.reference,trace=latest.trace||[];
  text('status',`状态：${latest.state}　原因：${latest.reason||'无'}　模式：${mode==='readonly'?'只读软件样例':'ROS软件仿真'}`);
  text('timing',`浏览器主线程最大间隔 ${(clientMaxGap).toFixed(1)}ms；挑战证据≤100ms`);
  if(c)text('pose',`软件车 X=${c.x.toFixed(3)}m　Y=${c.y.toFixed(3)}m　朝向=${(c.yaw*180/Math.PI).toFixed(1)}°`);
  text('countdown',`剩余 ${(latest.remaining_s??0).toFixed(1)} 秒`);
  const canvas=$('map'),box=canvas.getBoundingClientRect(),dpr=Math.min(2,devicePixelRatio||1),w=box.width,h=box.height;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const points=[...trace];for(const p of [c,g,r])if(p)points.push([p.x,p.y]);if(!points.length)return;
  let xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);const cx=(Math.max(...xs)+Math.min(...xs))/2,cy=(Math.max(...ys)+Math.min(...ys))/2;
  const hx=Math.max(1,(Math.max(...xs)-Math.min(...xs))/2+.3),hy=Math.max(1,(Math.max(...ys)-Math.min(...ys))/2+.3),scale=Math.min((w-50)/(2*hx),(h-55)/(2*hy));
  const point=(x,y)=>[w/2+(x-cx)*scale,h/2-(y-cy)*scale];
  ctx.strokeStyle='#93bad6';ctx.lineWidth=2;ctx.beginPath();trace.forEach((p,i)=>{const xy=point(...p);i?ctx.lineTo(...xy):ctx.moveTo(...xy);});ctx.stroke();
  for(const [p,color] of [[g,'#d38d1f'],[r,'#008b8b']])if(p){const [x,y]=point(p.x,p.y);ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,7,0,2*Math.PI);ctx.stroke();ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(...point(p.x+.23*Math.cos(p.yaw),p.y+.23*Math.sin(p.yaw)));ctx.stroke();}
  if(c){const [x,y]=point(c.x,c.y);ctx.save();ctx.translate(x,y);ctx.rotate(-c.yaw);ctx.fillStyle='#2986c1';ctx.fillRect(-.095*scale,-.105*scale,.19*scale,.21*scale);ctx.strokeStyle='white';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(.09*scale,0);ctx.stroke();ctx.restore();}
  ctx.fillStyle='#a23925';ctx.font='18px Microsoft YaHei,sans-serif';ctx.fillText('视图不是围栏；Vicon可见 ≠ 周围无障碍',12,26);
}
requestAnimationFrame(render);connect();
