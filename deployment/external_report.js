"use strict";
const records=JSON.parse(document.getElementById("report-data").textContent);
const platform=document.getElementById("platform"),slider=document.getElementById("time"),canvas=document.getElementById("track"),ctx=canvas.getContext("2d"),equal=document.getElementById("equal");
let azimuth=-0.7,elevation=0.45,zoom=1,timer=null,drag=null;
for(const [name,item] of Object.entries(records)){if(item.pose.length){const option=document.createElement("option");option.value=name;option.textContent=item.title;platform.appendChild(option);}}
function selected(){return records[platform.value];}
function index(){return Math.min(Math.max(0,Number(slider.value)),(selected()?.pose.length||1)-1);}
function nearest(rows,t){let best=null,delta=Infinity;for(const row of rows){const d=Math.abs(row[0]-t);if(d<delta){best=row;delta=d;}}return {row:best,delta};}
function snapshot(){const item=selected(),body=document.getElementById("pose");body.replaceChildren();if(!item){document.getElementById("clock").textContent="本次没有可显示的位置数据";return;}
 slider.max=item.pose.length-1;const k=index(),p=item.pose[k];document.getElementById("clock").textContent=(p[0]-item.pose[0][0]).toFixed(3)+" s · "+(k+1)+"/"+item.pose.length;
 const rows=[["局部位置 X / Y / Z (m)",p.slice(1,4).map(v=>v.toFixed(5)).join(" , ")],["局部速度 X / Y / Z (m/s)",p.slice(4,7).map(v=>v.toFixed(5)).join(" , ")]];
 const near=nearest(item.attitude,p[0]);if(near.row){const euler=item.attitude_type==="euler";rows.push([euler?"滚转 / 俯仰 / 偏航 (度)":"四元数 NB / wxyz",near.row.slice(1).map(v=>(euler?v*180/Math.PI:v).toFixed(6)).join(" , ")]);rows.push(["最近姿态采样的时间差 (s)",near.delta.toFixed(6)]);}
 for(const row of rows){const tr=document.createElement("tr");for(const value of row){const td=document.createElement("td");td.textContent=value;tr.appendChild(td);}body.appendChild(tr);}
 document.getElementById("plot-hint").textContent=item.directory+" · "+(equal.checked?"真实比例":"各坐标轴独立比例，非真实几何比例")+" · 橙色点为当前实际采样";
}
function draw(){const bounds=canvas.getBoundingClientRect(),w=bounds.width||900,h=bounds.height||430,dpr=window.devicePixelRatio||1;canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);const item=selected();if(!item)return;
 const points=item.pose.map(row=>row.slice(1,4)),lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];for(const p of points)for(let j=0;j<3;j++){lo[j]=Math.min(lo[j],p[j]);hi[j]=Math.max(hi[j],p[j]);}
 const spans=hi.map((v,j)=>Math.max(v-lo[j],0.01)),common=Math.max(...spans),center=hi.map((v,j)=>(v+lo[j])/2),scale=Math.min(w,h)*0.32*zoom;
 function normalize(p){return p.map((v,j)=>(v-center[j])/(equal.checked?common:spans[j])*1.6);}
 function project(p){const x=Math.cos(azimuth)*p[0]-Math.sin(azimuth)*p[1],y=Math.sin(azimuth)*p[0]+Math.cos(azimuth)*p[1];return [w/2+scale*x,h/2-scale*(Math.sin(elevation)*y+Math.cos(elevation)*p[2])];}
 function stroke(a,b,color,width=1){const p=project(a),q=project(b);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
 const corners=[];for(let x=0;x<2;x++)for(let y=0;y<2;y++)for(let z=0;z<2;z++)corners.push([x,y,z].map((v,j)=>normalize(v?hi:lo)[j]));
 for(let a=0;a<8;a++)for(let bit=0;bit<3;bit++){const b=a^(1<<bit);if(b>a)stroke(corners[a],corners[b],"#d4e1eb");}
 ctx.beginPath();for(let k=0;k<points.length;k++){const p=project(normalize(points[k]));if(k===0)ctx.moveTo(...p);else ctx.lineTo(...p);}ctx.strokeStyle="#236b9b";ctx.lineWidth=1.7;ctx.stroke();
 for(const [k,color,r]of [[0,"#168a57",4],[points.length-1,"#b43d4a",4],[index(),"#ed912d",6]]){const p=project(normalize(points[k]));ctx.beginPath();ctx.arc(...p,r,0,2*Math.PI);ctx.fillStyle=color;ctx.fill();}
 ctx.font='13px "Microsoft YaHei",sans-serif';for(let j=0;j<3;j++){const endpoint=lo.slice();endpoint[j]=hi[j];const p=project(normalize(endpoint));ctx.fillStyle=["#b14e47","#218169","#285eaa"][j];ctx.fillText(["前向 X","右向 Y","向上 Z"][j]+" "+lo[j].toFixed(2)+"～"+hi[j].toFixed(2)+" m",Math.max(8,Math.min(w-180,p[0]+8)),Math.max(18,Math.min(h-8,p[1])));}
}
function refresh(){snapshot();draw();}
function reset(){azimuth=-0.7;elevation=0.45;zoom=1;refresh();}
platform.addEventListener("change",()=>{slider.value=0;reset();});slider.addEventListener("input",refresh);equal.addEventListener("change",draw);document.getElementById("reset").onclick=reset;
canvas.addEventListener("pointerdown",event=>{drag=[event.clientX,event.clientY];canvas.setPointerCapture(event.pointerId);});
canvas.addEventListener("pointermove",event=>{if(!drag)return;azimuth+=(event.clientX-drag[0])*0.008;elevation=Math.max(-1.5,Math.min(1.5,elevation+(event.clientY-drag[1])*0.008));drag=[event.clientX,event.clientY];draw();});
for(const name of ["pointerup","pointercancel","lostpointercapture"])canvas.addEventListener(name,()=>{drag=null;});
canvas.addEventListener("wheel",event=>{event.preventDefault();zoom=Math.max(0.3,Math.min(3,zoom*Math.exp(-event.deltaY*0.001)));draw();},{passive:false});
document.getElementById("play").onclick=function(){if(timer){clearInterval(timer);timer=null;this.textContent="播放";return;}if(!selected())return;this.textContent="暂停";timer=setInterval(()=>{slider.value=Number(slider.value)>=Number(slider.max)?0:Math.min(Number(slider.value)+5,Number(slider.max));refresh();},50);};
window.addEventListener("resize",draw);refresh();
