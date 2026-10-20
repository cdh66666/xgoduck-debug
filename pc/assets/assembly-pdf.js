import {getDocument,GlobalWorkerOptions} from './vendor/pdfjs/pdf.js';
GlobalWorkerOptions.workerSrc='/assets/vendor/pdfjs/pdf.worker.js';
const host=document.querySelector('#pages'),picker=document.querySelector('#page'),zoom=document.querySelector('#zoom'),status=document.querySelector('#status');
let pdf,pages=[],generation=0,queue=Promise.resolve(),visible=new Set();
const scale=()=>zoom.value==='fit'?Math.max(.1,(host.clientWidth-40)/960):Number(zoom.value);
function render(index){
  const target=pages[index],version=generation;
  if(!target||target.dataset.rendered===String(version))return;
  target.dataset.rendered=String(version);
  queue=queue.then(async()=>{
    if(version!==generation)return;
    const page=await pdf.getPage(index+1),viewport=page.getViewport({scale:scale()}),ratio=Math.min(devicePixelRatio,1.5);
    const canvas=document.createElement('canvas');canvas.width=Math.ceil(viewport.width*ratio);canvas.height=Math.ceil(viewport.height*ratio);
    await page.render({canvasContext:canvas.getContext('2d'),viewport,transform:[ratio,0,0,ratio,0,0]}).promise;
    if(version===generation)target.replaceChildren(canvas);
  }).catch(error=>{status.textContent='PDF 显示失败：'+error.message;delete target.dataset.rendered;});
}
function resize(){
  if(!pdf)return;generation++;
  for(const page of pages){page.style.width=960*scale()+'px';page.style.height=540*scale()+'px';page.replaceChildren();}
  for(const index of visible)render(index);
}
const observer=new IntersectionObserver(entries=>{for(const entry of entries){const index=Number(entry.target.dataset.page);if(entry.isIntersecting){visible.add(index);render(index);}else visible.delete(index);}},{root:host,rootMargin:'600px 0px'});
picker.onchange=()=>pages[Number(picker.value)-1]?.scrollIntoView({block:'start'});
host.addEventListener('scroll',()=>{
  if(!pages.length)return;
  const top=host.getBoundingClientRect().top;
  const index=pages.findIndex(page=>page.getBoundingClientRect().bottom>top+60);
  if(index>=0)picker.value=String(index+1);
},{passive:true});
zoom.onchange=resize;
new ResizeObserver(resize).observe(host);
try{
  pdf=await getDocument({url:'/downloads/XGO-Duck-Assembly-Guide-ZH.pdf',isEvalSupported:false,useSystemFonts:true}).promise;
  for(let i=0;i<pdf.numPages;i++){
    picker.add(new Option(String(i+1),String(i+1)));const page=document.createElement('section');page.className='pdf-page';page.dataset.page=String(i);page.setAttribute('aria-label','PDF 第 '+(i+1)+' 页');host.append(page);pages.push(page);observer.observe(page);
  }
  status.textContent=pdf.numPages+' 页 · 中文版';document.body.dataset.pdfPages=String(pdf.numPages);resize();
}catch(error){status.textContent='PDF 读取失败';const message=document.createElement('p');message.className='error';message.textContent=error.message;host.append(message);}
