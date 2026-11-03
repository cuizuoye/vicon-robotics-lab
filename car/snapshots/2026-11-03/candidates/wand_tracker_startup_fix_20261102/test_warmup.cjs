'use strict';
const assert=require('assert'),PageWarmup=require('./page_warmup.js');let checks=0;
function test(fn){fn();checks++;}
test(()=>{const w=new PageWarmup();for(let i=0;i<10;i++)assert(!w.frame(i*33,2));assert(w.frame(600,2)===false);for(let t=633;t<1200;t+=33)w.frame(t,2);assert(w.ready);});
test(()=>{const w=new PageWarmup();for(let t=0;t<1000;t+=33)w.frame(t,140);assert(!w.ready,'slow canvas cannot connect');});
test(()=>{const w=new PageWarmup();for(let t=0;t<1000;t+=101)w.frame(t,1);assert(!w.ready,'stalled startup cannot connect');});
test(()=>{const w=new PageWarmup();w.frame(0,140);for(let t=33;t<700;t+=33)w.frame(t,1);assert(w.ready,'after warmup, connection is allowed');assert(!('nonce' in w));});
console.log(JSON.stringify({passed:true,checks,hardware_output:false,authority_ever_acquired:false}));
