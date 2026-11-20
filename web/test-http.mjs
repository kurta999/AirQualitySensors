import http from 'node:http';
import dns from 'node:dns/promises';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile,writeFile} from 'node:fs/promises';
const host='192.168.7.1',base=`http://${host}`;
function raw(path,options={}){
  return new Promise((resolve,reject)=>{
    const req=http.request({hostname:host,path,method:options.method??'GET',headers:options.headers??{},timeout:8000},response=>{
      const chunks=[];response.on('data',b=>chunks.push(b));response.on('end',()=>resolve({status:response.statusCode,headers:response.headers,body:Buffer.concat(chunks)}));
    });
    req.on('error',reject);req.on('timeout',()=>req.destroy(new Error('Device timeout')));
    req.end(options.body);
  });
}
async function config(){return (await fetch(`${base}/api/config`)).json();}
async function patch(values){
  const body=new URLSearchParams(Object.entries(values).map(([k,v])=>[k,String(v)])).toString();
  const r=await raw('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Air-Config':'1','Content-Length':Buffer.byteLength(body)},body});
  assert.equal(r.status,200,r.body.toString());return JSON.parse(r.body);
}
const report={};
const manifest=JSON.parse(await readFile(new URL('dist/manifest.json',import.meta.url)));
for(const file of manifest.files){
  const response=await raw(file.path);
  assert.equal(response.status,200);assert.equal(response.headers['content-encoding'],'gzip');
  assert.equal(createHash('sha256').update(response.body).digest('hex'),file.sha256);
}
report.flash_gzip_assets_match_build=manifest.files.length;
assert.equal((await dns.lookup('air.local',{family:4})).address,host);report.mdns_air_local=host;
const before=await config();
const common={'Content-Type':'application/x-www-form-urlencoded','Content-Length':'16'};
assert.equal((await raw('/api/config',{method:'POST',headers:common,body:'master_enabled=0'})).status,403);
assert.equal((await raw('/api/config',{method:'POST',headers:{...common,'X-Air-Config':'1',Origin:'http://other.example'},body:'master_enabled=0'})).status,403);
assert.equal((await raw('/api/config',{method:'POST',headers:{...common,'X-Air-Config':'1'},body:'master_enabled=2'})).status,400);
assert.equal((await raw('/api/config',{method:'POST',headers:{...common,'X-Air-Config':'1',Host:'other.example'},body:'master_enabled=0'})).status,403);
assert.equal((await config()).storage.sequence,before.storage.sequence);
assert.equal((await config()).config.master_enabled,1);report.invalid_and_cross_origin_writes_rejected=true;
let restored=false;
try{
  await patch({hostname:'air-check',scd_interval_s:3,light_gain:2,uv_integration:2,bsec_heat_offset:100});
  for(let n=0;n<8;n++){
    try{assert.equal((await dns.lookup('air-check.local',{family:4})).address,host);break;}
    catch(error){if(n===7)throw error;await new Promise(r=>setTimeout(r,1000));}
  }
  assert.equal((await raw('/',{headers:{Host:'air-check.local'}})).status,200);
  await new Promise(r=>setTimeout(r,9000));
  const live=await(await fetch(`${base}/api/measurements`)).json();
  assert.equal(live.fresh_mask,59);assert.equal(live.values.length,21);
  const applied=await config();assert.equal(applied.config.scd_interval_s,3);
  report.renamed_mdns_and_sensor_settings=true;
  await patch(Object.fromEntries(['hostname','scd_interval_s','light_gain','uv_integration','bsec_heat_offset'].map(key=>[key,before.config[key]])));
  restored=true;
}finally{
  if(!restored)await patch(Object.fromEntries(['hostname','scd_interval_s','light_gain','uv_integration','bsec_heat_offset'].map(key=>[key,before.config[key]])));
}
// Restoring SCD30 settings restarts its cycle; let real readings return before
// handing the board to the next transport test.
await new Promise(r=>setTimeout(r,7000));
for(let n=0;n<15;n++){
  const live=await(await fetch(`${base}/api/measurements`)).json();
  if(live.fresh_mask===59)break;
  if(n===14)throw new Error('Sensors did not settle after restoring settings');
  await new Promise(r=>setTimeout(r,1000));
}
// Four simultaneous flash/API transfers exercise response ownership and lwIP
// send backpressure without exceeding the documented four-connection limit.
await Promise.all(['/app.js','/style.css','/api/measurements','/api/history?after=0'].map(async path=>{
  const response=await raw(path);assert.equal(response.status,200);assert(response.body.length>0);
}));
report.parallel_transfers=4;
let cursor=0,records=0;
for(let page=0;page<20;page++){
  const response=await raw(`/api/history?after=${cursor}`),b=response.body;
  assert.equal(response.status,200);assert.equal(b.readUInt32LE(0),0x31524941);
  const count=b.readUInt32LE(16);assert(count<=32);assert.equal(b.length,32+96*count);
  if(!count)break;
  cursor=(b.readUInt32LE(28)+count-1)>>>0;records+=count;if(count<32)break;
}
assert(records>0);report.history_records_checked=records;
report.settings_restored=true;
await writeFile(new URL('../build/nucleo-usb/dashboard-http-test.json',import.meta.url),JSON.stringify(report,null,2));
console.log(JSON.stringify(report,null,2));
