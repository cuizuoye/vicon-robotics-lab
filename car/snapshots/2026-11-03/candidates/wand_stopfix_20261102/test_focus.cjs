'use strict';
// Actual client source under a stub DOM, not a physical-browser acceptance.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert'),{EventEmitter}=require('events');
const ready=path.resolve(process.argv[2]),out=path.resolve(process.argv[3]);fs.mkdirSync(out,{recursive:true});
const {createAudit,valid}=require(path.join(ready,'focus_audit.cjs'));
let checks=0;
function test(name,fn){fn();checks++;console.log('PASS '+name);}
function client(focused=true,hidden=false){
  const handlers={},docHandlers={},sent=[],audits=[],elements=new Map(),state={focused,hidden,now:0};
  const window={innerWidth:1280,innerHeight:720,addEventListener:(k,f)=>handlers[k]=f};
  const document={get hidden(){return state.hidden;},get visibilityState(){return state.hidden?'hidden':'visible';},
    hasFocus:()=>state.focused,fonts:{ready:Promise.resolve()},activeElement:{tagName:'CANVAS'},fullscreenElement:null,
    getElementById:id=>{if(!elements.has(id))elements.set(id,{textContent:'',hidden:false,value:'0',addEventListener(){}});return elements.get(id);},
    addEventListener:(k,f)=>docHandlers[k]=f};
  const intervals=[];
  const context=vm.createContext({window,document,location:{hash:'#test-token',host:'127.0.0.1'},performance:{now:()=>state.now},
    WebSocket:{OPEN:1},setInterval:f=>intervals.push(f),requestAnimationFrame(){},console,
    fetch:(url,args)=>{audits.push({url,body:JSON.parse(args.body)});return Promise.resolve();},
    __socket:{readyState:1,bufferedAmount:0,send:x=>sent.push(JSON.parse(x))}});
  for(const f of ['input_guard.js','focus_evidence.js','app.js'])vm.runInContext(fs.readFileSync(path.join(ready,'web',f),'utf8'),context);
  vm.runInContext('ws=__socket;lastView=0;guard.begin(0)',context);
  return {handlers,docHandlers,sent,audits,state,context,window,document,intervals};
}
for(const trusted of [false,true])test('event alone cannot stop focused visible page trusted='+trusted,()=>{
  const c=client();c.handlers.blur({isTrusted:trusted,target:c.window});
  assert.equal(c.sent.length,0);assert.equal(c.audits.length,1);assert.equal(c.audits[0].body.record.focused,true);
  assert.equal(vm.runInContext('guard.last',c.context),0);assert.equal(vm.runInContext('events.length',c.context),0);
});
test('descendant blur does not stop page',()=>{let c=client();c.handlers.blur({isTrusted:true,target:{}});assert.equal(c.sent.length,0);assert.equal(c.audits[0].body.record.target,'element');});
for(const trusted of [false,true])test('actual unfocused state stops immediately trusted='+trusted,()=>{
  let c=client(false);c.handlers.blur({isTrusted:trusted,target:c.window});assert.equal(c.sent[0].reason,'WINDOW_BLUR');
});
test('hidden page still stops',()=>{let c=client(true,true);c.docHandlers.visibilitychange({isTrusted:true,target:c.document});assert.equal(c.sent[0].reason,'WINDOW_HIDDEN');});
for(const kind of ['resize','fullscreenchange'])test(kind+' does not stop visible focused page',()=>{
  let c=client();(kind==='resize'?c.handlers:c.docHandlers)[kind]({isTrusted:true,target:kind==='resize'?c.window:c.document});assert.equal(c.sent.length,0);
});
test('trusted top-level pagehide still stops',()=>{let c=client();c.handlers.pagehide({isTrusted:true,target:c.window});assert.equal(c.sent[0].reason,'PAGE_CLOSED');});
test('synthetic pagehide alone is not a lifecycle proof',()=>{let c=client();c.handlers.pagehide({isTrusted:false,target:c.window});assert.equal(c.sent.length,0);});
test('20ms periodic guard still catches actual loss without blur',()=>{let c=client();c.state.focused=false;c.state.now=20;c.intervals[0]();assert.equal(c.sent[0].reason,'WINDOW_NOT_FOREGROUND');});
test('100ms UI deadline unchanged',()=>{let c=client();c.state.now=101;c.intervals[0]();assert.equal(c.sent[0].reason,'CLIENT_LOOP_GAP');});
test('100ms view deadline unchanged',()=>{let c=client();c.state.now=80;c.intervals[0]();c.state.now=101;c.intervals[0]();assert.equal(c.sent[0].reason,'VIEW_STALE_100MS');});
test('challenge rechecks focus rather than diagnostic state',()=>{let c=client();c.state.focused=false;vm.runInContext('reply({protocol:"WAND_BROWSER_V1",ttl_ms:100,nonce:"n"},0)',c.context);assert.equal(c.sent[0].reason,'WINDOW_NOT_FOREGROUND');});
test('regaining focus never auto-enables or resumes',()=>{let c=client(false);c.handlers.blur({isTrusted:true,target:c.window});c.state.focused=true;c.handlers.focus({isTrusted:true,target:c.window});assert.equal(c.sent.length,1);assert(vm.runInContext('ended',c.context));});
test('diagnostics are capped and never generate pulse/key events',()=>{
  let c=client();for(let i=0;i<300;i++)c.handlers.resize({isTrusted:true,target:c.window});
  assert.equal(c.audits.length,256);assert.equal(c.sent.length,0);assert.equal(vm.runInContext('events.length',c.context),0);
});
test('untrusted keyboard event still cannot start',()=>{let c=client();c.docHandlers.keydown({isTrusted:false,code:'Space'});assert.equal(vm.runInContext('events.length',c.context),0);});
test('Esc and stop button still stop',()=>{
  let c=client();c.docHandlers.keydown({isTrusted:true,code:'Escape',preventDefault(){}});assert.equal(c.sent[0].reason,'ESC');
  c=client();vm.runInContext('$("stop").onclick()',c.context);assert.equal(c.sent[0].reason,'STOP_BUTTON');
});
async function main(){
  let clock=0;const file=path.join(out,'focus_audit.jsonl'),audit=createAudit(file,'test-token',()=>clock);
  const record=n=>({sequence:n,kind:'blur',at_ms:n,trusted:false,target:'window',focused:true,hidden:false,
    visibility:'visible',active_tag:'CANVAS',width:1280,height:720,fullscreen:false});
  test('strict audit record schema',()=>{
    assert(valid(record(1)));assert(!valid({...record(1),held:true}));assert(!valid({...record(1),at_ms:NaN}));
    assert(!valid({...record(1),kind:'pulse'}));assert(!valid({...record(1),focused:1}));
  });
  test('token, schema and monotonic audit sequence required',()=>{
    assert.equal(audit.accept({token:'wrong',record:record(1)}),403);
    assert.equal(audit.accept({token:'test-token',record:{...record(1),events:[]}}),400);
    assert.equal(audit.accept({token:'test-token',record:record(1)}),204);
    assert.equal(audit.accept({token:'test-token',record:record(1)}),400);
  });
  test('audit bounded at 10 records per second',()=>{
    for(let i=2;i<=30;i++)assert.equal(audit.accept({token:'test-token',record:record(i)}),204);
    assert.equal(audit.stats().rows,10);assert(audit.stats().dropped>=20);
  });
  function request(method,origin,body,type='application/json'){
    let code;const req=new EventEmitter();req.method=method;req.headers={origin,'content-type':type};req.resume=()=>{};
    const res={writeHead:x=>{code=x;},end(){}};audit.handle(req,res,x=>x==='http://127.0.0.1:1234');
    if(code===undefined){req.emit('data',Buffer.from(body));req.emit('end');}return code;
  }
  test('diagnostic HTTP origin/method/content type/body bounds',()=>{
    assert.equal(request('POST','http://evil','{}'),403);assert.equal(request('GET','http://127.0.0.1:1234','{}'),403);
    assert.equal(request('POST','http://127.0.0.1:1234','{}','text/plain'),403);
    assert.equal(request('POST','http://127.0.0.1:1234','x'.repeat(2049)),413);
    assert.equal(request('POST','http://127.0.0.1:1234','not json'),400);
    clock++;assert.equal(request('POST','http://127.0.0.1:1234',JSON.stringify({token:'test-token',record:record(31)})),204);
  });
  for(let n=32;n<=1000;n++){clock++;audit.accept({token:'test-token',record:record(n)});await new Promise(setImmediate);}
  test('audit hard cap and no authority state',()=>{assert.equal(audit.stats().rows,256);assert(audit.stats().diagnostic_only);});
  const closed=new Promise(resolve=>setTimeout(resolve,100));audit.close();await closed;
  test('audit contains no token and no lease/pulse',()=>{
    const text=fs.readFileSync(file,'utf8');assert(!text.includes('test-token'));assert(!text.includes('"held"'));assert(!text.includes('"pulse"'));
    assert.equal(text.trim().split('\n').length,256);
  });
  const result={passed:true,checks,actual_client_source:true,stub_DOM_not_actual_browser:true,hardware_output:false,authority_ever_acquired:false};
  fs.writeFileSync(path.join(out,'RESULT.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
}
main().catch(e=>{console.error(e);process.exitCode=1;});
