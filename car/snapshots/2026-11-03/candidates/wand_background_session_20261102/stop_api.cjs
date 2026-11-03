'use strict';
const crypto=require('crypto');
function createStopAPI(token,stop,originOK){
 let requests=0;
 return function handle(req,res){
  const end=code=>{res.writeHead(code,{'Content-Type':'application/json','Cache-Control':'no-store'});res.end(JSON.stringify({stop_requested:code===202}));};
  if(req.method!=='POST'||!originOK(req.headers.origin)||req.headers['content-type']!=='application/json'){end(403);req.resume();return;}
  let raw='',tooLarge=false;
  req.on('data',b=>{raw+=b;if(Buffer.byteLength(raw)>512){tooLarge=true;raw='';}});
  req.on('end',()=>{
   if(tooLarge){end(413);return;}
   let d;try{d=JSON.parse(raw);}catch{end(400);return;}
   const got=typeof d?.token==='string'?Buffer.from(d.token):Buffer.alloc(0),want=Buffer.from(token);
   if(got.length!==want.length||!crypto.timingSafeEqual(got,want)){end(403);return;}
   if(Object.keys(d).sort().join(',')!=='reason,token,type'||d.type!=='stop'||!['LOCAL_STOP','ESTOP','PAGE_CLOSED'].includes(d.reason)){end(400);return;}
   // Idempotent and bounded; never a START or a fresh motion lease.
   if(requests++<8)stop(d.reason);
   end(202);
  });
 };
}
module.exports={createStopAPI};
