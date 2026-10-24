import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const source=fs.readFileSync(new URL('../pc/assets/official-models.js',import.meta.url),'utf8');
const settle=()=>new Promise(r=>setImmediate(r));
async function harness(){
 const nodes=new Map(),events={},messages=[];let focus=0;
 class Element{
  constructor(){this.dataset={};this.children=[];this.attrs={};this.classList={add(){},toggle(){}};this.contentWindow={postMessage:m=>messages.push(m)};}
  setAttribute(k,v){this.attrs[k]=v;}getAttribute(k){return this.attrs[k]??null;}hasAttribute(k){return k in this.attrs;}
  set src(v){this.attrs.src=v;}append(v){this.children.push(v);}focus(){focus++;}addEventListener(){}
 }
 const $=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
 const document={body:$('body'),hidden:false,getElementById:$,createElement:()=>new Element(),querySelector:()=>null,querySelectorAll:()=>$('model-list').children,addEventListener:(n,f)=>events[n]=f};
 const window={parent:{postMessage(){}},addEventListener:(n,f)=>events[n]=f};
 vm.runInNewContext(source,{window,document,location:{origin:'http://offline'},setTimeout,clearTimeout});await settle();
 const send=(data,from=$('official-model-preview').contentWindow)=>events.message({origin:'http://offline',source:from,data});
 const state=(running=false)=>send({type:'duck-simulation-preview-state',model_id:'official-walk',model_sha:'a'.repeat(64),policy_enabled:running,paused:!running});
 const ack=()=>send({type:'duck-simulation-control-result',model_id:'official-walk',ok:true});
 state();
 return {messages,focus:()=>focus,state,ack,key:(key,tagName='BODY',repeat=false)=>events.keydown({key,code:key===' '?'Space':key,target:{tagName},repeat,preventDefault(){}}),parent:data=>send(data,window.parent)};
}
test('first Space starts the selected model; a second pauses after acknowledgement',async()=>{
 const h=await harness();h.key(' ');
 assert.equal(h.messages.at(-1).command,'run');assert.equal(h.focus(),1);
 const count=h.messages.length;h.key(' ', 'BODY',true);h.key(' ');assert.equal(h.messages.length,count,'in-flight toggle is not queued');
 h.state(true);h.ack();h.key(' ');assert.equal(h.messages.at(-1).command,'pause');
});
test('parent and library shortcuts share the model binding and preserve text entry',async()=>{
 const h=await harness();const count=h.messages.length;h.key(' ','INPUT');assert.equal(h.messages.length,count);
 h.parent({type:'bench-models-shortcut',version:1,key:'space'});assert.equal(h.messages.at(-1).expected_sha,'a'.repeat(64));h.ack();
 h.key('r');assert.equal(h.messages.at(-1).command,'reset');h.ack();h.key('Escape');assert.equal(h.messages.at(-1).command,'stop');h.ack();
 h.parent({type:'bench-models-visibility',visible:false});const hidden=h.messages.length;h.key(' ');assert.equal(h.messages.length,hidden);
});
