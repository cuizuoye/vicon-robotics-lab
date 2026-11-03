'use strict';
// Bounded, non-authoritative diagnostics. Never forwards to the ROS worker.
const fs=require('fs'),crypto=require('crypto');
const KINDS=new Set(['focus','blur','visibilitychange','resize','fullscreenchange','pagehide']);
const KEYS='active_tag,at_ms,focused,fullscreen,height,hidden,kind,sequence,target,trusted,visibility,width';
function valid(v){
  return v&&Object.keys(v).sort().join(',')===KEYS&&KINDS.has(v.kind)
    &&Number.isSafeInteger(v.sequence)&&v.sequence>0&&v.sequence<=1000000
    &&Number.isFinite(v.at_ms)&&v.at_ms>=0
    &&['focused','hidden','trusted','fullscreen'].every(k=>typeof v[k]==='boolean')
    &&['window','document','element','unknown'].includes(v.target)
    &&['visible','hidden','prerender','unknown'].includes(v.visibility)
    &&typeof v.active_tag==='string'&&/^[A-Z0-9_-]{0,32}$/.test(v.active_tag)
    &&['width','height'].every(k=>Number.isInteger(v[k])&&v[k]>=0&&v[k]<=32768);
}
function createAudit(file,token,clock=()=>Number(process.hrtime.bigint())/1e9){
  const stream=fs.createWriteStream(file,{flags:'wx',mode:0o600});
  let failed=false,rows=0,lastSequence=0,windowAt=-Infinity,inWindow=0,dropped=0;
  stream.on('error',()=>{failed=true;});
  function accept(body){
    if(!body||Object.keys(body).sort().join(',')!=='record,token'||typeof body.token!=='string')return 403;
    const got=Buffer.from(body.token),want=Buffer.from(token);
    if(got.length!==want.length||!crypto.timingSafeEqual(got,want))return 403;
    if(!valid(body.record)||body.record.sequence<=lastSequence)return 400;
    lastSequence=body.record.sequence;
    const now=clock();if(now-windowAt>=1){windowAt=now;inWindow=0;}
    if(failed||rows>=256||++inWindow>10||stream.writableLength>8192){dropped++;return 204;}
    stream.write(JSON.stringify({received_steady:now,diagnostic_only:true,record:body.record})+'\n');rows++;
    return 204;
  }
  function handle(req,res,originOK){
    if(req.method!=='POST'||!originOK(req.headers.origin)||req.headers['content-type']!=='application/json'){
      req.resume();res.writeHead(403);res.end();return;
    }
    let bytes=0,chunks=[],done=false;
    req.on('data',chunk=>{
      if(done)return;bytes+=chunk.length;
      if(bytes>2048){done=true;chunks=[];res.writeHead(413);res.end();return;}
      chunks.push(chunk);
    });
    req.on('end',()=>{
      if(done)return;done=true;let body;
      try{body=JSON.parse(Buffer.concat(chunks).toString());}catch{res.writeHead(400);res.end();return;}
      res.writeHead(accept(body));res.end();
    });
    req.on('error',()=>{done=true;});
  }
  return {accept,handle,close:()=>stream.end(),stats:()=>({rows,dropped,failed,diagnostic_only:true})};
}
module.exports={createAudit,valid};
