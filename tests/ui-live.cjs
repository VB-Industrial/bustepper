// Run with Playwright available (e.g. NODE_PATH=/tmp/bustepper-browser/node_modules).
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({headless:true,args:['--no-sandbox']});
 const page=await browser.newPage({viewport:{width:1440,height:1150}});
 const errors=[],writes=[];page.on('pageerror',e=>errors.push(e.message));
 const fake={connected:true,status:[0,0,0,1000,0,0],position:2.5,velocity:0,steps:203718,
  settings:{ihold:0,irun:0,global_scaler:33,gear_ratio:10,acceleration:0.5,vel_set:0.25},sample_id:0,instance:'test-1'};
 await page.routeWebSocket('**/ws',async ws=>{
  const sendTelemetry=()=>{
   fake.sample_id++;
   ws.send(JSON.stringify({type:'telemetry',...fake,samples:[{id:fake.sample_id,time:fake.sample_id*.05,position:fake.position,velocity:fake.velocity}]}));
  };
  const timer=setInterval(sendTelemetry,50);timer.unref();
  ws.onClose(()=>clearInterval(timer));
  ws.onMessage(raw=>{
   const body=JSON.parse(raw);writes.push(body);
   if(body.name==='arm')fake.status[0]=body.value;
   if(body.name==='move_rad')fake.status[1]=1;
   if(body.name==='stop')fake.status[1]=0;
   if(body.name in fake.settings)fake.settings[body.name]=body.value;
   ws.send(JSON.stringify({type:'command_result',sequence:body.sequence,ok:true,status:fake.status,value:[body.value]}));
  });
  sendTelemetry();
 });
 await page.goto(process.env.BUSTEPPER_UI_URL||'http://192.168.1.104:8088');await page.waitForFunction(()=>state.connected);
 assert.equal(writes.length,0,'Loading the page must not write to CAN');
 assert.equal(await page.locator('.demo').count(),0,'Demo badge removed');
 assert.equal(await page.locator('.temperature').count(),0);
 await page.locator('#arm').click();await page.waitForFunction(()=>state.armed&&busy===0);
 await page.locator('#positive').click();await page.waitForTimeout(1100);
 assert.ok(writes.filter(x=>x.name==='move_rad').length>=2,'Velocity command refreshes');
 assert.equal(await page.evaluate(()=>state.position),2.5,'Position comes from telemetry, no simulation');
 await page.locator('#velocity-stop').click();await page.waitForFunction(()=>busy===0);
 const afterStop=writes.filter(x=>x.name==='move_rad').length;
 await page.waitForTimeout(650);
 assert.equal(writes.filter(x=>x.name==='move_rad').length,afterStop,'Stop cancels velocity refresh');
 await page.locator('[data-tab=position]').click();
 await page.locator('#position-speed').fill('0.2');await page.locator('#position-speed').dispatchEvent('change');
 await page.waitForFunction(()=>busy===0);
 assert.ok(writes.some(x=>x.name==='vel_set'&&x.value===0.2),'Position speed uses vel_set');
 await page.locator('#position-input').fill('0.5');
 await page.locator('#go-position').click();await page.waitForFunction(()=>busy===0);
 assert.ok(writes.some(x=>x.name==='pos_set'&&x.value===0.5));
 await page.locator('#position-stop').click();await page.waitForFunction(()=>busy===0);
 await page.locator('#irun').fill('5');await page.locator('#irun').dispatchEvent('change');await page.waitForFunction(()=>busy===0);
 assert.ok(writes.some(x=>x.name==='irun'&&x.value===5));
 await page.locator('[data-tab=velocity]').click();await page.locator('#positive').click();await page.waitForFunction(()=>busy===0);
 fake.connected=false;await page.waitForFunction(()=>!state.connected);
 assert.ok(await page.locator('#positive').isDisabled());
 fake.connected=true;fake.instance='test-2';fake.sample_id=0;
 await page.waitForFunction(()=>state.connected);const before=writes.length;
 await page.waitForTimeout(750);
 assert.equal(writes.length,before,'Reconnection must never resume a command');
 await page.setViewportSize({width:390,height:844});
 assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
 assert.deepEqual(errors,[]);await browser.close();
 console.log('Live UI checks passed: no startup writes, RPC commands, keepalive, both stops, telemetry, offline/reconnect, mobile.');
})().catch(e=>{console.error(e);process.exit(1);});
