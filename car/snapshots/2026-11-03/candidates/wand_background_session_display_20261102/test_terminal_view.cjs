'use strict';
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=path.resolve(process.argv[2]);let checks=0;
for(const confirmed of [true,false,null]){
 const sent=[],els=new Map();class WS{static OPEN=1;constructor(){this.readyState=1;}send(v){sent.push(JSON.parse(v));}}
 const document={getElementById:id=>{if(!els.has(id))els.set(id,{textContent:'',hidden:false});return els.get(id);},
  hasFocus:()=>true,hidden:false,fonts:{ready:Promise.resolve()},addEventListener(){}};
 const c=vm.createContext({window:{addEventListener(){}},document,WebSocket:WS,location:{hash:'#test',host:'127.0.0.1'},
   performance:{now:()=>0},URL,fetch:()=>Promise.resolve(),requestAnimationFrame(){},console});
 vm.runInContext(fs.readFileSync(path.join(root,'web/page_warmup.js'),'utf8'),c);
 vm.runInContext(fs.readFileSync(path.join(root,'web/app.js'),'utf8'),c);
 vm.runInContext("resourcesReady=pageReady=true;connect();ws.onmessage({data:JSON.stringify({kind:'hello',mode:'rehearsal',authority_mode:'WAND_BACKEND_SESSION_V1',owner:'one'})});",c);
 vm.runInContext("viewWs.onmessage({data:JSON.stringify({kind:'view',view_sequence:1,state:'ACTIVE',car:{x:0,y:0,yaw:0}})});",c);
 const result={kind:'result',result:{reason:confirmed?'CLIENT_ESC':'OPERATOR_ESTOP',stop_evidence:confirmed===null?undefined:{stop_confirmed:confirmed,gate_output_zero:true}}};
 c.__result=JSON.stringify(result);vm.runInContext('ws.onmessage({data:__result})',c);
 const expected=confirmed?'STOPPED_CONFIRMED':'STOP_UNCONFIRMED';
 assert.equal(vm.runInContext('latest.state',c),expected);
 vm.runInContext("viewWs.onmessage({data:JSON.stringify({kind:'view',view_sequence:2,state:'ACTIVE'})});",c);
 assert.equal(vm.runInContext('latest.state',c),expected,'late view resurrected ACTIVE');
 assert(vm.runInContext('resultSeen&&ended',c));assert(sent.some(v=>v.type==='ack'&&v.sequence===2));checks++;
}
console.log(JSON.stringify({passed:true,checks,actual_client_source:true,stub_DOM:true,hardware_output:false}));
