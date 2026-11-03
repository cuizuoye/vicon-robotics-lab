'use strict';
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
let checks=0;function test(name,fn){fn();checks++;console.log('PASS '+name);}
function make(focused=true,hidden=false){
 const state={focused,hidden,now:0},handlers={},docHandlers={},sent=[],elements=new Map();
 const window={addEventListener:(k,f)=>handlers[k]=f};
 const document={hasFocus:()=>state.focused,get hidden(){return state.hidden;},fonts:{ready:Promise.resolve()},
  getElementById:id=>{if(!elements.has(id))elements.set(id,{textContent:'',hidden:false});return elements.get(id);},
  addEventListener:(k,f)=>docHandlers[k]=f};
 const c=vm.createContext({window,document,location:{host:'127.0.0.1',hash:'#test'},performance:{now:()=>state.now},
  WebSocket:{OPEN:1},__socket:{readyState:1,send:x=>sent.push(JSON.parse(x))},URL,
  fetch:()=>Promise.resolve(),console});
 vm.runInContext(fs.readFileSync(path.join(__dirname,'page_warmup.js'),'utf8'),c);
 vm.runInContext(fs.readFileSync(path.join(__dirname,'client.js'),'utf8'),c);
 vm.runInContext("ws=__socket;latest={state:'ARMED',prepare_phase:'READY'};lastView=0;offer={nonce:'fresh',received:0};",c);
 const event=(code='Space',more={})=>({code,isTrusted:true,repeat:false,preventDefault(){},...more});
 return {state,handlers,docHandlers,sent,c,event};
}
test('blur does not stop active backend session',()=>{const x=make();vm.runInContext('startSent=true',x.c);x.state.focused=false;x.handlers.blur();assert.equal(x.sent.length,0);assert(!vm.runInContext('ended',x.c));});
test('hidden page does not stop backend session',()=>{const x=make();x.state.hidden=true;x.docHandlers.visibilitychange();assert.equal(x.sent.length,0);assert(!vm.runInContext('ended',x.c));});
test('untrusted start is ignored',()=>{const x=make();x.docHandlers.keydown(x.event('Space',{isTrusted:false}));x.docHandlers.keyup(x.event('Space',{isTrusted:false}));assert.equal(x.sent.length,0);});
test('fresh completed click emits exactly one explicit start',()=>{const x=make();x.docHandlers.keydown(x.event());x.docHandlers.keyup(x.event());assert.deepEqual(x.sent,[{type:'command',action:'start',nonce:'fresh',sequence:1}]);});
test('release without press cannot start',()=>{const x=make();x.docHandlers.keyup(x.event());assert.equal(x.sent.length,0);});
test('auto-repeat cannot duplicate start',()=>{const x=make();x.docHandlers.keydown(x.event());x.docHandlers.keydown(x.event('Space',{repeat:true}));x.docHandlers.keyup(x.event());assert.equal(x.sent.length,1);});
test('blur cancels unfinished click rather than deferring start',()=>{const x=make();x.docHandlers.keydown(x.event());x.state.focused=false;x.handlers.blur();x.state.focused=true;x.docHandlers.keyup(x.event());assert.equal(x.sent.length,0);});
test('visibility loss cancels unfinished click',()=>{const x=make();x.docHandlers.keydown(x.event());x.state.hidden=true;x.docHandlers.visibilitychange();x.state.hidden=false;x.docHandlers.keyup(x.event());assert.equal(x.sent.length,0);});
test('hidden/unfocused synthetic key path cannot request start',()=>{for(const args of [[false,false],[true,true]]){const x=make(...args);x.docHandlers.keydown(x.event());x.docHandlers.keyup(x.event());assert.equal(x.sent.length,0);}});
test('stale view cannot start',()=>{const x=make();x.state.now=501;x.docHandlers.keydown(x.event());x.docHandlers.keyup(x.event());assert.equal(x.sent.length,0);});
test('holding key too long cannot start',()=>{const x=make();x.docHandlers.keydown(x.event());x.state.now=751;vm.runInContext('lastView=751;offer.received=751',x.c);x.docHandlers.keyup(x.event());assert.equal(x.sent.length,0);});
test('second space always requests stop',()=>{const x=make();x.docHandlers.keydown(x.event());x.docHandlers.keyup(x.event());x.docHandlers.keydown(x.event());assert.equal(x.sent[1].reason,'SECOND_SPACE');});
test('Esc remains stop',()=>{const x=make();x.docHandlers.keydown(x.event('Escape'));assert.equal(x.sent[0].reason,'ESC');});
test('stop button and estop remain stop-only',()=>{for(const [id,reason] of [['stop','STOP_BUTTON'],['estop','ESTOP']]){const x=make();vm.runInContext(`$('${id}').onclick()`,x.c);assert.equal(x.sent[0].reason,reason);}});
test('focus return cannot revive ended session',()=>{const x=make();x.docHandlers.keydown(x.event('Escape'));x.handlers.focus();x.docHandlers.keydown(x.event());x.docHandlers.keyup(x.event());assert.equal(x.sent.length,1);});
test('page close still withdraws session',()=>{const x=make();x.handlers.pagehide();assert.equal(x.sent[0].reason,'PAGE_CLOSED');});
console.log(JSON.stringify({passed:true,checks,stub_DOM:true,hardware_output:false}));
