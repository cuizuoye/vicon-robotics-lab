'use strict';
const token=location.hash.slice(1),$=id=>document.getElementById(id);
let navigateAttempt=null,initial=true,polling=false;
const names={IDLE:'等待检查',CHECKING:'启动检查',DISABLED_ZERO:'小车未使能握手',VICON:'Vicon接收',WARMUP:'定位与安全门自检',BROWSER_READY:'自检通过',STOPPING:'正在停车',STOPPED:'本轮已结束',BLOCKED:'检查未通过'};
async function api(route,method='GET'){
 const response=await fetch(route,{method,headers:{'X-Launch-Token':token,...(method==='POST'?{'Content-Type':'application/json'}:{})},...(method==='POST'?{body:JSON.stringify({action:'check'})}:{})});
 const value=await response.json();if(!response.ok)throw Error(value.error||'启动器连接失败');return value;
}
async function start(){
 $('retry').disabled=true;const value=await api('/check','POST');navigateAttempt=value.attempt;show(value);
}
function show(v){
 $('phase').textContent=names[v.phase]||v.phase;$('message').textContent=v.message;
 $('retry').disabled=v.busy||!v.retry_allowed;
 $('detail').textContent=JSON.stringify({phase:v.phase,attempt:v.attempt,log:v.log,stop_confirmed:v.stop_confirmed,exit_code:v.exit_code},null,2);
 if(v.phase==='BROWSER_READY'&&navigateAttempt===v.attempt&&v.runtime_url){
  // Same tab, never a popup that can steal/lose the foreground input owner.
  navigateAttempt=null;location.replace(v.runtime_url);
 }
}
async function poll(){
 if(polling)return;polling=true;
 try{
  const v=await api('/status');show(v);
  if(initial){initial=false;if(v.phase==='IDLE'&&v.attempt===0)await start();}
 }catch(error){$('phase').textContent='启动器连接异常';$('message').textContent=error.message+'；不会启动或恢复运动。';}
 finally{polling=false;}
}
$('retry').onclick=()=>start().catch(e=>{$('message').textContent=e.message;});
poll();setInterval(poll,300);
