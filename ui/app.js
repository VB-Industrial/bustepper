'use strict';
const $ = id => document.getElementById(id);
const state = {connected:false,armed:false,mode:'hold',position:0,velocity:0,target:0,
 speed:0.1,positionSpeed:0.25,gear:10,acceleration:0.5,stepsUnit:false,steps:0};
const history=[], historyLength=400;
const charts=[{id:'output-speed-chart',key:'velocity',target:true,color:'#d6a848',floor:0.2},
 {id:'output-position-chart',key:'position',color:'#7db5ff',floor:0.5},
 {id:'chart',key:'rpm',target:true,color:'#d6a848',floor:2}];
const session=globalThis.crypto?.randomUUID?.()||('browser-'+Date.now()+'-'+Math.random().toString(36).slice(2));
let socket=null;
const pendingCommands=new Map();
let bridgeInstance=null;
let sequence=0,lastSample=0,lastGood=0,lastFrame=0,lastError='',wasConnected=false;
let velocityDirection=0,generation=0,refreshing=false,commandQueue=Promise.resolve(),busy=0;
const pendingSettings=new Set();
function log(text,type='СТЕНД'){
 const row=document.createElement('div');row.className='log-entry';
 const time=document.createElement('time');time.textContent=new Date().toLocaleTimeString('ru-RU');
 const tag=document.createElement('span');tag.className='log-type';tag.textContent=type;
 const message=document.createElement('span');message.textContent=text;
 row.append(time,tag,message);$('log').prepend(row);
 while($('log').children.length>50)$('log').lastChild.remove();
}
function stepsPerRad(){return 51200*state.gear/(2*Math.PI);}
function cancelVelocity(){velocityDirection=0;generation++;state.target=0;}
function updateControls(){
 const online=state.connected;
 document.body.dataset.armed=String(online&&state.armed);
 $('arm').textContent=state.armed?'Выключить':'Включить';
 $('arm').disabled=!online||busy>0;
 $('arm-status').textContent=!online?'Нет связи с приводом':state.armed?'Драйвер включён':'Драйвер выключен';
 $('arm-indicator').classList.toggle('on',online&&state.armed);
 $('mode-status').textContent=!online?'Ожидание телеметрии CAN':state.mode==='velocity'?'Движение со скоростью':state.mode==='position'?'Позиционный режим':Math.abs(state.velocity)>0.00001?'Торможение':'Ожидание команды';
 ['positive','negative','go-position'].forEach(id=>$(id).disabled=!online||!state.armed||busy>0);
 $('gear').disabled=!online||state.armed||Math.abs(state.velocity)>0.00001||busy>0;
 $('zero').disabled=!online||Math.abs(state.velocity)>0.00001||busy>0;
 ['irun','ihold','scaler','acceleration','position-speed'].forEach(id=>$(id).disabled=!online);
 $('positive').classList.toggle('selected',online&&state.mode==='velocity'&&velocityDirection>0);
 $('negative').classList.toggle('selected',online&&state.mode==='velocity'&&velocityDirection<0);
 $('connection-status').textContent=online?'CAN0 · узел 42 · телеметрия Cyphal 20 Гц':'Нет свежей телеметрии · команды движения заблокированы';
}
function requestCommand(name,value){
 return new Promise((resolve,reject)=>{
  if(!socket||socket.readyState!==WebSocket.OPEN){reject(Error('Нет WebSocket-соединения с Pi'));return;}
  const id=++sequence;
  const timeout=setTimeout(()=>{pendingCommands.delete(id);reject(Error('Нет подтверждения команды'));},3000);
  pendingCommands.set(id,{resolve,reject,timeout});
  socket.send(JSON.stringify({type:'command',name,value,session,sequence:id}));
 });
}
function write(name,value,{quiet=false,valid=()=>true}={}){
 if(!quiet){busy++;pendingSettings.add(name);updateControls();}
 const result=commandQueue.catch(()=>{}).then(async()=>{
  if(!valid())return null;
  if(!state.connected)throw Error('Нет связи с приводом');
  return requestCommand(name,value);
 });
 commandQueue=result;
 return result.then(data=>{if(data&&!quiet)log(`${name} = ${value} · подтверждено приводом`,'КОМАНДА');return data;})
 .catch(error=>{cancelVelocity();log(`${name}: ${error.message}`,'ОШИБКА');throw error;})
 .finally(()=>{if(!quiet){busy--;pendingSettings.delete(name);}updateControls();});
}
function fire(name,value,options){write(name,value,options).catch(()=>{});}
function stop(){cancelVelocity();fire('stop',1);}
$('stop').onclick=stop;$('velocity-stop').onclick=stop;$('position-stop').onclick=stop;
$('arm').onclick=()=>{cancelVelocity();fire('arm',state.armed?0:1);};
function syncSpeed(n,source){
 const physical=state.stepsUnit&&source==='input'?n/stepsPerRad():n;
 if(!Number.isFinite(physical))return;
 state.speed=Math.max(0,Math.min(1,physical));
 $('velocity-input').value=state.stepsUnit?Math.round(state.speed*stepsPerRad()):state.speed.toFixed(2);
 $('velocity-range').value=state.speed;
}
$('velocity-input').onchange=e=>syncSpeed(Number(e.target.value),'input');
$('velocity-range').oninput=e=>syncSpeed(Number(e.target.value),'range');
$('velocity-unit').onclick=()=>{
 state.stepsUnit=!state.stepsUnit;$('velocity-input').max=state.stepsUnit?Math.round(stepsPerRad()):1;
 $('velocity-input').step=state.stepsUnit?1:0.01;
 $('velocity-input').nextElementSibling.textContent=state.stepsUnit?'мкш/с':'рад/с';
 $('velocity-unit').textContent=state.stepsUnit?'⇄ рад/с':'⇄ микрошаг/с';syncSpeed(state.speed,'range');
};
for(const [id,sign]of[['negative',-1],['positive',1]])$(id).onclick=()=>{
 if(!state.connected||!state.armed)return;
 cancelVelocity();velocityDirection=sign;const token=generation;
 state.target=sign*state.speed;
 fire('move_rad',state.target,{valid:()=>generation===token});
};
setInterval(()=>{
 if(!velocityDirection||!state.connected||!state.armed||refreshing||busy)return;
 const token=generation;state.target=velocityDirection*state.speed;refreshing=true;
 write('move_rad',state.target,{quiet:true,valid:()=>generation===token}).catch(()=>{}).finally(()=>refreshing=false);
},500);
document.querySelectorAll('[data-tab]').forEach(button=>button.onclick=()=>{
 document.querySelectorAll('[data-tab]').forEach(b=>b.classList.toggle('active',b===button));
 $('velocity-view').hidden=button.dataset.tab!=='velocity';$('position-view').hidden=button.dataset.tab!=='position';
});
$('go-position').onclick=()=>{
 const n=Number($('position-input').value);if(!Number.isFinite(n))return;
 const steps=$('position-unit').value==='steps';
 if(steps&&!Number.isInteger(n)){log('Позиция в микрошагax должна быть целой','ОШИБКА');return;}
 if(Math.abs(steps?n:n*stepsPerRad())>2147483647){log('Позиция выходит за диапазон int32','ОШИБКА');return;}
 cancelVelocity();fire(steps?'tmc_pos_set':'pos_set',n);
};
$('zero').onclick=()=>{cancelVelocity();fire('zero_set',1);};
for(const id of ['irun','ihold']){
 $(id).oninput=()=>{$(id+'-value').innerHTML=`${$(id).value} <span>/ 31</span>`;};
 $(id).onchange=()=>fire(id,Number($(id).value));
}
$('scaler').onchange=()=>{
 const n=Number($('scaler').value);
 if(!Number.isInteger(n)||(n!==0&&(n<32||n>255))){log('GLOBAL_SCALER: допустимы 0 или 32…255','ОШИБКА');return;}
 fire('global_scaler',n);
};
$('gear').onchange=()=>fire('gear_ratio',Number($('gear').value));
$('acceleration').onchange=()=>fire('acceleration',Number($('acceleration').value));
$('position-speed').onchange=()=>fire('vel_set',Number($('position-speed').value));
$('clear-log').onclick=()=>$('log').replaceChildren();
window.addEventListener('keydown',e=>{if(e.code==='Escape')stop();});
function endVelocity(){
 if(!velocityDirection)return;cancelVelocity();
 if(socket?.readyState===WebSocket.OPEN)
  socket.send(JSON.stringify({type:'command',name:'stop',value:1,session,sequence:++sequence}));
}
window.addEventListener('pagehide',endVelocity);
document.addEventListener('visibilitychange',()=>{if(document.hidden)endVelocity();});
function setField(id,value,name){
 if(document.activeElement!==$(id)&&!pendingSettings.has(name))$(id).value=value;
}
function applySettings(settings){
 if(!settings)return;
 for(const id of ['irun','ihold'])if(settings[id]!==undefined){
  setField(id,settings[id],id);$(id+'-value').innerHTML=`${$(id).value} <span>/ 31</span>`;
 }
 if(settings.global_scaler!==undefined){setField('scaler',settings.global_scaler,'global_scaler');$('scaler-percent').textContent=((settings.global_scaler||256)/256*100).toFixed(1)+'%';}
 if(settings.gear_ratio!==undefined){state.gear=settings.gear_ratio;setField('gear',state.gear,'gear_ratio');$('gear-label').textContent=state.gear+':1';}
 if(settings.acceleration!==undefined){state.acceleration=settings.acceleration;setField('acceleration',state.acceleration,'acceleration');}
 if(settings.vel_set!==undefined){state.positionSpeed=settings.vel_set;setField('position-speed',state.positionSpeed,'vel_set');$('position-speed-label').textContent=state.positionSpeed.toFixed(2);}
}
function render(){
 const fresh=state.connected;
 $('position-value').innerHTML=fresh?state.position.toFixed(4)+' <small>рад</small>':'— <small>рад</small>';
 $('velocity-value').innerHTML=fresh?state.velocity.toFixed(3)+' <small>рад/с</small>':'— <small>рад/с</small>';
 $('live-position').innerHTML=fresh?state.position.toFixed(3)+' <small>рад</small>':'— <small>рад</small>';
 $('live-velocity').innerHTML=fresh?state.velocity.toFixed(3)+' <small>рад/с</small>':'— <small>рад/с</small>';
 $('steps-value').textContent=fresh?state.steps.toLocaleString('ru-RU'):'—';
 $('rotor').setAttribute('transform',`rotate(${state.position*180/Math.PI} 120 105)`);
 updateControls();charts.forEach(drawChart);
}
function applyTelemetry(data){
  if(data.instance&&data.instance!==bridgeInstance){
   if(bridgeInstance){cancelVelocity();history.length=0;log('CAN-сервер перезапущен · команды движения сброшены','СВЯЗЬ');}
   bridgeInstance=data.instance;lastSample=0;
  }
  state.connected=!!data.connected;
  if(state.connected){
   lastGood=performance.now();state.armed=!!data.status[0];state.mode=['hold','velocity','position'][data.status[1]]||'hold';
   if(!state.armed)cancelVelocity();
   state.position=data.position;state.velocity=data.velocity;state.steps=data.steps;
   applySettings(data.settings);
   for(const sample of data.samples||[]){
    if(sample.id<=lastSample)continue;
    lastSample=sample.id;history.push({velocity:sample.velocity,position:sample.position,gear:state.gear,target:state.target,time:sample.time});
   }
   if(history.length>historyLength)history.splice(0,history.length-historyLength);
   if(history.length)lastFrame=performance.now();
   if(data.status[2]&&String(data.status[2])!==lastError){lastError=String(data.status[2]);log('Привод сообщает ошибку '+data.status[2],'ОШИБКА');}
   if(!data.status[2])lastError='';
  }else{cancelVelocity();}
  if(state.connected!==wasConnected){log(state.connected?'CAN0 · узел 42 · подписка Planar.0.1, subject 1042':'Телеметрия недоступна. Движение автоматически не возобновляется.',state.connected?'СВЯЗЬ':'ОШИБКА');wasConnected=state.connected;}
 render();
}
function disconnect(message){
 state.connected=false;cancelVelocity();
 for(const item of pendingCommands.values()){clearTimeout(item.timeout);item.reject(Error('Связь с Pi потеряна'));}
 pendingCommands.clear();
 if(wasConnected){log(message,'ОШИБКА');wasConnected=false;}
 render();
}
function connect(){
 const transport=location.protocol==='https:'?'wss:':'ws:';
 const current=new WebSocket(`${transport}//${location.host}/ws`);socket=current;
 current.onmessage=event=>{
  if(current!==socket)return;
  try{
   const data=JSON.parse(event.data);
   if(data.type==='telemetry')applyTelemetry(data);
   if(data.type==='command_result'){
    const item=pendingCommands.get(data.sequence);if(!item)return;
    clearTimeout(item.timeout);pendingCommands.delete(data.sequence);
    if(!data.ok){item.reject(Error(data.error||'Команда отклонена'));return;}
    if(data.status){state.armed=!!data.status[0];state.mode=['hold','velocity','position'][data.status[1]]||'hold';}
    item.resolve(data);
   }
  }catch(error){disconnect('Некорректные данные сервера: '+error.message);current.close();}
 };
 current.onclose=()=>{
  if(current!==socket)return;
  disconnect('WebSocket отключён. Движение автоматически не возобновляется.');
  setTimeout(connect,1000);
 };
 current.onerror=()=>current.close();
}
function drawChart(chart){
 const canvas=$(chart.id),rect=canvas.getBoundingClientRect();if(!rect.width||!rect.height)return;
 const dpr=window.devicePixelRatio||1;
 if(canvas.width!==Math.round(rect.width*dpr)||canvas.height!==Math.round(rect.height*dpr)){
  canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);
 }
 const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);
 const w=rect.width,h=rect.height,l=40,r=4,t=8,b=13,pw=w-l-r,ph=h-t-b;
 ctx.clearRect(0,0,w,h);
 const newest=history.at(-1)?.time||0;
 const rightTime=newest+Math.max(0,(performance.now()-lastFrame)/1000);
 const xAt=s=>l+pw*(s.time-(rightTime-20))/20;
 const value=s=>chart.key==='rpm'?s.velocity*s.gear*60/(2*Math.PI):s[chart.key];
 const target=s=>chart.key==='rpm'?s.target*s.gear*60/(2*Math.PI):s.target;
 let min=0,max=chart.floor;
 for(const s of history){min=Math.min(min,value(s),chart.target?target(s):0);max=Math.max(max,value(s),chart.target?target(s):0);}
 const range=Math.max(chart.floor,max-min);max+=range*.13;min-=range*.13;
 const y=n=>t+(max-n)/(max-min)*ph;
 ctx.font='9px Segoe UI';ctx.textAlign='right';ctx.fillStyle='#758293';
 ctx.strokeStyle='#2b3541';ctx.lineWidth=1;
 for(let i=0;i<5;i++){
  const n=min+(max-min)*i/4,yy=y(n);ctx.beginPath();ctx.moveTo(l,yy);ctx.lineTo(w-r,yy);ctx.stroke();ctx.fillText(n.toFixed(chart.key==='rpm'?1:2),l-7,yy+3);
 }
 for(let i=0;i<5;i++){const x=l+pw*i/4;ctx.beginPath();ctx.moveTo(x,t);ctx.lineTo(x,h-b);ctx.stroke();}
 if(chart.target){ctx.strokeStyle='#738296';ctx.setLineDash([4,4]);ctx.beginPath();history.forEach((s,i)=>{const x=xAt(s);i?ctx.lineTo(x,y(target(s))):ctx.moveTo(x,y(target(s)));});ctx.stroke();ctx.setLineDash([]);}
 ctx.beginPath();history.forEach((s,i)=>{const x=xAt(s);i?ctx.lineTo(x,y(value(s))):ctx.moveTo(x,y(value(s)));});
 ctx.strokeStyle=chart.color;ctx.lineWidth=2;ctx.stroke();
 const last=history.at(-1);if(!last)return;ctx.beginPath();ctx.arc(xAt(last),y(value(last)),3,0,2*Math.PI);ctx.fillStyle=chart.color;ctx.fill();
}

updateControls();render();log('Подключение к Cyphal: телеметрия subject 1042, команды через регистры','СВЯЗЬ');connect();
setInterval(()=>{
 if(state.connected&&performance.now()-lastGood>750){state.connected=false;cancelVelocity();render();}
},100);
window.addEventListener('resize',()=>charts.forEach(drawChart));
