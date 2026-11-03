'use strict';
// Reuse all existing 24 stop/focus VM cases, with the new pre-connection class loaded.
const fs=require('fs'),path=require('path');
const original='/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_stopfix_20261102/test_focus.cjs';
let source=fs.readFileSync(original,'utf8');
const old="['input_guard.js','focus_evidence.js','app.js']";
if(source.split(old).length!==2)throw Error('unexpected parent test layout');
source=source.replace(old,"['input_guard.js','focus_evidence.js','page_warmup.js','app.js']");
module._compile(source,__filename);
