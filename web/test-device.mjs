/* Real-board browser checks. No simulated sensor values are used. */
import { chromium } from 'playwright-core';
import { mkdir,writeFile,readFile } from 'node:fs/promises';
import assert from 'node:assert/strict';
const base=process.env.AIR_DEVICE_URL??'http://192.168.7.1';
const output=new URL('../build/nucleo-usb/',import.meta.url);
await mkdir(output,{recursive:true});
const browser=await chromium.launch({channel:'chrome',headless:true});
const page=await browser.newPage({viewport:{width:1440,height:1050},deviceScaleFactor:1});
const errors=[],external=[];
page.on('pageerror',error=>errors.push(error.message));
page.on('console',message=>{if(message.type()==='error')errors.push(message.text());});
page.on('request',request=>{if(!request.url().startsWith(base)&&!request.url().startsWith('data:'))external.push(request.url());});
const checks={};
try{
  await page.goto(base,{waitUntil:'domcontentloaded'});
  await page.getByText('Connected via USB',{exact:true}).waitFor({timeout:25000});
  await page.locator('.reading').first().filter({hasText:/\d/}).waitFor({timeout:10000});
  assert.equal(await page.locator('canvas').count(),22);
  assert.equal(await page.locator('.sensor-state').filter({hasText:'Online'}).count(),4);
  checks.live_sensors=4;checks.individual_charts=22;
  await page.screenshot({path:new URL('dashboard-desktop.png',output).pathname.replace(/^\/(\w:)/,'$1')});
  await page.getByRole('button',{name:'Light & UV',exact:true}).click();
  assert.equal(await page.locator('#metric-grid article:visible').count(),6);
  await page.getByRole('button',{name:'All measurements',exact:true}).click();
  await page.getByRole('button',{name:'1 hour',exact:true}).click();
  assert(await page.getByRole('button',{name:'1 hour',exact:true}).evaluate(e=>e.classList.contains('selected')));
  checks.filter_and_range=true;
  await page.getByRole('link',{name:'Configuration',exact:true}).click();
  await page.locator('#settings-page').waitFor({state:'visible'});
  const before=await(await page.request.get(`${base}/api/config`)).json();
  const restore=process.argv.includes('--restore');
  if(!restore){try{await readFile(new URL('dashboard-original-config.json',output));}catch{await writeFile(new URL('dashboard-original-config.json',output),JSON.stringify(before.config,null,2));}}
  let expectedHistory=5,expectedSend=2000;
  if(restore){
    const original=JSON.parse(await readFile(new URL('dashboard-original-config.json',output),'utf8'));
    expectedHistory=Number(original.history_interval_s);expectedSend=Number(original.send_interval_ms);
    await page.locator('#field-history_interval_s').selectOption(String(original.history_interval_s));
    await page.locator('#field-send_interval_ms').fill(String(original.send_interval_ms));
  }else{
    await page.locator('#field-history_interval_s').selectOption('5');
    await page.locator('#field-send_interval_ms').fill('2000');
  }
  await page.getByRole('button',{name:'Save to device',exact:true}).click();
  await page.getByText('Saved to device flash',{exact:true}).waitFor();
  const after=await(await page.request.get(`${base}/api/config`)).json();
  assert.equal(after.config.history_interval_s,expectedHistory);
  assert.equal(after.config.send_interval_ms,expectedSend);
  assert(after.storage.sequence>=before.storage.sequence);
  checks.settings_saved_sequence=after.storage.sequence;
  if(!restore){
    await page.locator('#field-master_enabled').uncheck();
    await page.getByRole('button',{name:'Save to device',exact:true}).click();
    await page.getByText('Saved to device flash',{exact:true}).waitFor();
    await page.waitForTimeout(5500);
    const paused=await(await page.request.get(`${base}/api/measurements`)).json();assert.equal(paused.fresh_mask,0);
    await page.locator('#field-master_enabled').check();
    await page.getByRole('button',{name:'Save to device',exact:true}).click();
    await page.getByText('Saved to device flash',{exact:true}).waitFor();
    for(let n=0;n<15;n++){
      const resumed=await(await page.request.get(`${base}/api/measurements`)).json();
      if(resumed.fresh_mask===59)break;
      await page.waitForTimeout(1000);
      if(n===14)throw new Error('Sensors did not resume after master enable');
    }
    checks.master_disable_and_resume=true;
  }
  await page.screenshot({path:new URL('dashboard-settings.png',output).pathname.replace(/^\/(\w:)/,'$1')});
  await page.setViewportSize({width:390,height:844});
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.screenshot({path:new URL('dashboard-mobile-settings.png',output).pathname.replace(/^\/(\w:)/,'$1')});
  await page.getByRole('link',{name:'Overview',exact:true}).click();
  await page.waitForTimeout(2500);
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.screenshot({path:new URL('dashboard-mobile.png',output).pathname.replace(/^\/(\w:)/,'$1')});
  checks.mobile_width=390;
  const downloadPromise=page.waitForEvent('download');
  await page.getByRole('button',{name:'Export CSV',exact:true}).click();
  const download=await downloadPromise;assert(download.suggestedFilename().endsWith('.csv'));checks.csv_export=true;
  assert.deepEqual(external,[]);assert.deepEqual(errors,[]);
  checks.no_external_requests=true;checks.browser_errors=errors;
  await writeFile(new URL(restore?'dashboard-browser-restored.json':'dashboard-browser-test.json',output),JSON.stringify(checks,null,2));
  console.log(JSON.stringify(checks,null,2));
}finally{await browser.close();}
