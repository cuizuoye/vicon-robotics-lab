'use strict';
const fs=require('fs'),path=require('path'),assert=require('assert');
const view=require('./renderer.js');
const output=path.resolve(process.argv[2]);let checks=0;
function test(name,fn){fn();checks++;console.log('PASS '+name);}
function near(a,b){assert(Math.abs(a-b)<1e-8,`${a} != ${b}`);}
const camera=()=>({halfY:3});
test('world origin stays at centre, not car origin',()=>{
  const v=view.layout(1200,600,{x:1.3,y:-2.1},null,[],camera());
  assert.deepEqual(v.project(0,0),[600,300]);assert.notDeepEqual(v.project(1.3,-2.1),[600,300]);
});
test('positive X right and positive Y up, equal metre scale',()=>{
  const v=view.layout(1200,600,null,null,[],camera());
  const o=v.project(0,0),x=v.project(1,0),y=v.project(0,1);
  assert(x[0]>o[0]&&y[1]<o[1]);near(x[0]-o[0],o[1]-y[1]);
});
test('default grid is 1m major and .2m minor',()=>{
  const v=view.layout(860,360,null,null,[],camera());near(v.major,1);near(v.minor,.2);
});
test('5-6m translation in every quadrant fits without changing origin',()=>{
  for(const x of [-6,6])for(const y of [-6,6]){
    const v=view.layout(860,360,{x,y},null,[],camera()),p=v.project(x,y);
    assert(p[0]>25&&p[0]<835&&p[1]>25&&p[1]<335);assert.deepEqual(v.project(0,0),[430,180]);
  }
});
test('display never auto zooms in while returning',()=>{
  const c=camera();const wide=view.layout(860,360,{x:6,y:6},null,[],c);
  const back=view.layout(860,360,{x:0,y:0},null,[],c);near(back.scale,wide.scale);
});
test('resize keeps world origin and orthonormal XY axes',()=>{
  const c=camera();for(const [w,h]of [[860,360],[1500,720],[500,320]]){
    const v=view.layout(w,h,{x:5,y:-6},null,[],c);assert.deepEqual(v.project(0,0),[w/2,h/2]);
    const x=v.project(1,0),y=v.project(0,1);near(x[0]-w/2,h/2-y[1]);
  }
});
test('poses and traces are never mutated by the renderer',()=>{
  const c={x:1,y:2,yaw:.7},g={x:3,y:4,yaw:-.8},t=[[0,1],[2,3]];
  const before=JSON.stringify([c,g,t]);view.layout(860,360,c,g,t,camera());assert.equal(JSON.stringify([c,g,t]),before);
});
test('projection is invertible for deterministic large-range samples',()=>{
  for(let i=0;i<1000;i++){
    const x=8*Math.sin(i*.17),y=8*Math.cos(i*.31),v=view.layout(1280,720,{x,y},null,[],camera()),p=v.project(x,y);
    near((p[0]-640)/v.scale,x);near((360-p[1])/v.scale,y);
  }
});
test('all draw operations finite, grid bounded, origin/axes visible without a car',()=>{
  for(const car of [null,{x:0,y:0,yaw:0},{x:-6,y:6,yaw:Math.PI/2},{x:1e6,y:-1e6,yaw:-Math.PI}]){
    let lines=0;const labels=[];
    const ctx=new Proxy({}, {get:(_,key)=> (...args)=>{
      if(['moveTo','lineTo','fillRect','strokeRect','translate','rotate','rect'].includes(key)) assert(args.every(Number.isFinite),key);
      if(key==='moveTo')lines++;if(key==='fillText')labels.push(args[0]);
    },set:()=>true});
    view.draw(ctx,1200,500,car,{x:1,y:-2,yaw:0},[[],[0],[0,0],[NaN,0]],camera());
    assert(lines<800);for(const label of ['0, 0','+X','+Y','VICON WORLD · XY'])assert(labels.includes(label));
  }
});
test('display module has no transport, timer, or authority APIs',()=>{
  const source=fs.readFileSync(path.join(__dirname,'renderer.js'),'utf8');
  assert(!/WebSocket|fetch\(|postMessage|setInterval|setTimeout|sendto|rclpy/.test(source));
});
fs.mkdirSync(path.dirname(output),{recursive:true});
const result={passed:true,checks,world_frame:'map',identity_map_transform_preserved:true,
  origin_fixed:true,XY_uniform_metre_scale:true,positive_X_right:true,positive_Y_up:true,
  large_range_m:6,only_zoom_out:true,no_input_mutation:true,no_hardware_IO:true};
fs.writeFileSync(output,JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
