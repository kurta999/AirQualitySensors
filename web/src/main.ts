import { Chart, LineController, LineElement, PointElement, LinearScale, Tooltip, Filler } from 'chart.js';
import { decodeHistory, rowValue, type HistoryRow, columnSources } from './protocol';
Chart.register(LineController,LineElement,PointElement,LinearScale,Tooltip,Filler);

type ConfigValue = string | number;
type Config = Record<string,ConfigValue>;
interface DeviceInfo {
  config: Config;
  capabilities: { sensor_mask: number; bsec: number };
  storage: { sequence:number; history_capacity:number; history_bytes:number; interval_s:number; retention_s:number;
    history_count:number; ram1_static_bytes:number; ram1_total_bytes:number; ram2_total_bytes:number;
    rtos_heap_free_bytes:number; firmware_bytes:number };
  network: { ip:string; client_ip:string; transport:string };
}
interface Live {
  uptime_ms:number; valid_mask:number; fresh_mask:number; config_revision:number; values:(number|null)[];
  iaq:{ index:number; accuracy:number; stabilized:number; run_in:number; timestamp_ms:number };
  usb:{ dhcp_replies:number; tcp_connected:number; records_sent:number; acknowledged_bytes:number };
  history_count:number;
}
interface Metric { column:number; title:string; unit:string; sensor:string; group:'air'|'climate'|'light'|'particle'; color:string; digits:number; detail:string; icon:string }
const blue='#5b83ee', green='#42ad99', purple='#9281db', amber='#e7b26b';
const metrics:Metric[] = [
  {column:13,title:'Air quality index',unit:'IAQ',sensor:'BME680 · BSEC',group:'air',color:blue,digits:0,detail:'Bosch BSEC air quality index',icon:'≈'},
  {column:0,title:'Carbon dioxide',unit:'ppm',sensor:'SCD30',group:'climate',color:green,digits:0,detail:'Direct CO₂ measurement',icon:'CO₂'},
  {column:1,title:'Temperature',unit:'°C',sensor:'SCD30',group:'climate',color:amber,digits:1,detail:'Ambient temperature',icon:'°'},
  {column:2,title:'Relative humidity',unit:'%',sensor:'SCD30',group:'climate',color:purple,digits:1,detail:'Moisture in the air',icon:'◉'},
  {column:3,title:'Sensor temperature',unit:'°C',sensor:'BME680',group:'climate',color:amber,digits:2,detail:'Raw sensor temperature',icon:'°'},
  {column:4,title:'Sensor humidity',unit:'%',sensor:'BME680',group:'climate',color:purple,digits:2,detail:'Raw relative humidity',icon:'◉'},
  {column:5,title:'Air pressure',unit:'hPa',sensor:'BME680',group:'climate',color:blue,digits:1,detail:'Absolute atmospheric pressure',icon:'≋'},
  {column:6,title:'Gas resistance',unit:'Ω',sensor:'BME680',group:'air',color:green,digits:0,detail:'Metal oxide gas sensor',icon:'∿'},
  {column:14,title:'Static air quality',unit:'IAQ',sensor:'BME680 · BSEC',group:'air',color:blue,digits:0,detail:'Static IAQ estimate',icon:'≈'},
  {column:15,title:'CO₂ equivalent',unit:'ppm',sensor:'BME680 · BSEC',group:'air',color:green,digits:0,detail:'Gas-derived estimate, separate from SCD30 CO₂',icon:'CO₂'},
  {column:16,title:'Breath VOC equivalent',unit:'ppm',sensor:'BME680 · BSEC',group:'air',color:purple,digits:2,detail:'Estimated breath-related VOC equivalent',icon:'∿'},
  {column:17,title:'Compensated temperature',unit:'°C',sensor:'BME680 · BSEC',group:'climate',color:amber,digits:2,detail:'BSEC heat compensation applied',icon:'°'},
  {column:18,title:'Compensated humidity',unit:'%',sensor:'BME680 · BSEC',group:'climate',color:purple,digits:2,detail:'BSEC heat compensation applied',icon:'◉'},
  {column:12,title:'Illuminance',unit:'lx',sensor:'TCS34725',group:'light',color:amber,digits:0,detail:'Calculated from the color channels',icon:'☼'},
  {column:11,title:'Color temperature',unit:'K',sensor:'TCS34725',group:'light',color:blue,digits:0,detail:'Correlated color temperature',icon:'◌'},
  {column:7,title:'Ultraviolet',unit:'raw',sensor:'VEML6070',group:'light',color:purple,digits:0,detail:'Raw UV reading; not a calibrated UV index',icon:'☀'},
  {column:8,title:'Red channel',unit:'level',sensor:'TCS34725',group:'light',color:'#d9868f',digits:0,detail:'Red / clear channel × 255',icon:'R'},
  {column:9,title:'Green channel',unit:'level',sensor:'TCS34725',group:'light',color:green,digits:0,detail:'Green / clear channel × 255',icon:'G'},
  {column:10,title:'Blue channel',unit:'level',sensor:'TCS34725',group:'light',color:blue,digits:0,detail:'Blue / clear channel × 255',icon:'B'},
  {column:21,title:'IAQ accuracy',unit:'/ 3',sensor:'BME680 · BSEC',group:'air',color:blue,digits:0,detail:'0 learning · 1 low · 2 medium · 3 high',icon:'◎'},
  {column:22,title:'Heater stabilized',unit:'',sensor:'BME680 · BSEC',group:'air',color:green,digits:0,detail:'0 warming up · 1 stabilized',icon:'◷'},
  {column:23,title:'Run-in complete',unit:'',sensor:'BME680 · BSEC',group:'air',color:purple,digits:0,detail:'0 running in · 1 complete',icon:'◷'},
  {column:19,title:'Fine particles',unit:'µg/m³',sensor:'HPMA',group:'particle',color:amber,digits:0,detail:'PM2.5 mass concentration',icon:'◌'},
  {column:20,title:'Coarse particles',unit:'µg/m³',sensor:'HPMA',group:'particle',color:blue,digits:0,detail:'PM10 mass concentration',icon:'◌'}
];
const $ = <T extends HTMLElement = HTMLElement>(id:string):T => document.getElementById(id) as T;
const qa = <T extends Element = HTMLElement>(query:string):T[] => [...document.querySelectorAll<T>(query)];
let info:DeviceInfo|undefined, live:Live|undefined, history:HistoryRow[] = [], cursor=0;
let hours=24, category='all', dirty=false, saving=false, failed=0, lastRevision=0, lastHistory=0, initialized=false;
let visibleMetrics:Metric[] = [], pendingCharts = new Map<number,IntersectionObserverEntry>();
const charts=new Map<number,Chart<'line'>>();
const timeFormat = new Intl.DateTimeFormat(undefined,{hour:'2-digit',minute:'2-digit'});
let toastTimer:ReturnType<typeof setTimeout>;
function toast(text:string,error=false){ const t=$('toast'); t.textContent=text;t.classList.toggle('error',error);t.hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>t.hidden=true,5000); }
async function request(path:string,init?:RequestInit):Promise<Response>{
  const response=await fetch(path,{cache:'no-store',signal:AbortSignal.timeout(8000),...init});
  if(!response.ok){let message=`Device returned ${response.status}`;try{message=(await response.json()).error??message;}catch{}throw new Error(message);}
  return response;
}
function sourceFor(column:number){return column<21?columnSources[column]:5;}
function enabledMetric(metric:Metric){return !!(info?.capabilities.sensor_mask! & (1<<sourceFor(metric.column))) || (sourceFor(metric.column)===5 && !!info?.capabilities.bsec);}
function currentRow():HistoryRow|undefined {return live?{tick:live.uptime_ms,time:Date.now(),mask:live.fresh_mask,sequence:0,values:live.values,accuracy:live.iaq.accuracy,stabilized:live.iaq.stabilized,run_in:live.iaq.run_in}:undefined;}
function card(metric:Metric,summary=false):string{
  return `<article class="metric-card ${summary&&metric.column===13?'hero':''}" data-metric="${metric.column}" data-group="${metric.group}">
    <div class="card-label"><span>${metric.title}</span><span class="metric-icon" style="color:${metric.color}">${metric.icon}</span></div>
    <div class="metric-value"><span class="reading empty">—</span><span class="unit">${metric.unit}</span></div>
    <div class="card-detail">${metric.detail}</div><div class="card-chart"><canvas id="chart-${metric.column}" role="img" aria-label="${metric.title} history"></canvas></div>
    <div class="card-source"><span>${metric.sensor}</span><span class="card-freshness">WAITING</span></div></article>`;
}
// All strings interpolated into cards and fields below are source-controlled;
// device/user values are assigned through textContent or form.value.
function renderMetrics(){
  for(const chart of charts.values())chart.destroy();charts.clear();
  visibleMetrics=metrics.filter(enabledMetric);
  $('summary').innerHTML=visibleMetrics.slice(0,4).map(m=>card(m,true)).join('');
  $('metric-grid').innerHTML=visibleMetrics.slice(4).map(m=>card(m)).join('');
  const observer=new IntersectionObserver(entries=>{
    for(const entry of entries){const column=Number((entry.target as HTMLElement).dataset.metric);if(entry.isIntersecting){pendingCharts.set(column,entry);drawMetric(column);}}
  },{rootMargin:'200px'});
  qa<HTMLElement>('[data-metric]').forEach(e=>observer.observe(e));
  const sensors=[['SCD30','I²C1',1],['BME680','SPI2',2],['TCS34725','I²C1',16],['VEML6070','I²C1',8]] as const;
  $('sensor-strip').innerHTML='<span class="sensor-strip-label">CONNECTED SENSORS</span>'+sensors.filter(s=>info!.capabilities.sensor_mask&s[2]).map(s=>`<div class="sensor-status stale" data-sensor="${s[2]}"><span class="status-icon"></span><strong>${s[0]}</strong><small>${s[1]}</small><span class="sensor-state">Waiting</span></div>`).join('');
  applyFilters();
}
function drawMetric(column:number){
  if(!pendingCharts.has(column))return;
  const metric=visibleMetrics.find(m=>m.column===column);if(!metric)return;
  const cutoff=Date.now()-hours*3600000;
  const rows=history.filter(r=>r.time>=cutoff);
  const now=currentRow();if(now)rows.push(now);
  const points=rows.map(row=>({x:row.time,y:rowValue(row,column)}));
  const dotRadius=points.filter(p=>p.y!==null).length<=2?2.5:0;
  let chart=charts.get(column);
  if(!chart){
    const canvas=$<HTMLCanvasElement>(`chart-${column}`);if(!canvas)return;
    const summary=column===13||column===0||column===1||column===2;
    chart=new Chart(canvas,{type:'line',data:{datasets:[{data:points,borderColor:metric.color,backgroundColor:metric.color+'10',borderWidth:1.7,pointRadius:dotRadius,pointHoverRadius:4,fill:true,tension:.25,spanGaps:false}]},options:{responsive:true,maintainAspectRatio:false,animation:false,parsing:false,normalized:true,interaction:{intersect:false,mode:'nearest'},plugins:{legend:{display:false},tooltip:{displayColors:false,backgroundColor:'#273b60',padding:10,titleFont:{size:11},bodyFont:{size:12},callbacks:{title:items=>timeFormat.format(items[0].parsed.x??0),label:item=>`${Number(item.parsed.y).toLocaleString(undefined,{maximumFractionDigits:metric.digits})} ${metric.unit}`}}},scales:{x:{type:'linear',grid:{display:false},border:{display:false},display:!summary,ticks:{font:{size:9},color:'#a6b1c3',maxTicksLimit:4,maxRotation:0,callback:v=>timeFormat.format(Number(v))}},y:{type:'linear',display:!summary,border:{display:false},grid:{color:'#eff2f8',drawTicks:false},ticks:{font:{size:9},color:'#a6b1c3',maxTicksLimit:3,padding:8},grace:'15%'}}}});
    charts.set(column,chart);
  }else{chart.data.datasets[0].data=points;chart.data.datasets[0].pointRadius=dotRadius;chart.update('none');}
}
function updateReadings(){
  const row=currentRow();if(!row||!live)return;
  for(const metric of visibleMetrics){
    const article=document.querySelector<HTMLElement>(`[data-metric="${metric.column}"]`);if(!article)continue;
    const value=rowValue(row,metric.column),reading=article.querySelector<HTMLElement>('.reading')!;
    reading.textContent=value===null?'—':value.toLocaleString(undefined,{maximumFractionDigits:metric.digits,minimumFractionDigits:metric.digits});
    reading.classList.toggle('empty',value===null);
    article.querySelector<HTMLElement>('.card-freshness')!.textContent=value===null?'NO FRESH DATA':'LIVE';
    if(metric.column===13){
      article.querySelector<HTMLElement>('.card-detail')!.textContent=value===null?'Waiting for BSEC measurements':live.iaq.accuracy===0?'Learning your environment · accuracy 0/3':`BSEC accuracy ${live.iaq.accuracy}/3 · ${live.iaq.run_in?'run-in complete':'running in'}`;
    }
    drawMetric(metric.column);
  }
  qa<HTMLElement>('[data-sensor]').forEach(element=>{
    const mask=Number(element.dataset.sensor),enabled=Number(info?.config.master_enabled)&&((Number(info?.config.sensor_mask)&mask)!==0),fresh=!!(live!.fresh_mask&mask);
    element.classList.toggle('stale',!fresh);element.querySelector('.sensor-state')!.textContent=!enabled?'Disabled':fresh?'Online':'Waiting';
  });
  $('last-update').textContent='Updated just now';
  $('history-note').textContent=history.length<2?'Collecting history. Your first reading is recorded; the next point appears at the sampling interval.':`${history.length} recorded points · history stays on the device while powered · CSV includes all measurements`;
}
function applyFilters(){
  qa<HTMLElement>('#metric-grid [data-metric]').forEach(e=>e.hidden=category!=='all'&&e.dataset.group!==category);
  qa<HTMLButtonElement>('[data-group]').filter(e=>e.tagName==='BUTTON').forEach(e=>e.classList.toggle('selected',e.dataset.group===category));
  for(const column of pendingCharts.keys())drawMetric(column);
}

interface Field {key:string;label:string;type?:'toggle'|'text'|'number'|'select';min?:number;max?:number;step?:number;help?:string;options?:[number,string][];pattern?:string;maxLength?:number}
interface FieldGroup {title:string;description:string;fields:Field[]}
const groups:FieldGroup[]=[
  {title:'Station',description:'A familiar name, and one switch for all sensor activity.',fields:[
    {key:'device_name',label:'Station name',type:'text',maxLength:31,pattern:'[A-Za-z0-9 _\\-]+',help:'Letters, numbers, spaces, hyphens, and underscores.'},
    {key:'hostname',label:'Local hostname',type:'text',maxLength:31,pattern:'[a-z0-9]+(?:[a-z0-9\\-]*[a-z0-9])?',help:'Open hostname.local in your browser. Default: air.local.'},
    {key:'master_enabled',label:'Enable all sensor activity',type:'toggle'},
    {key:'sensor_scd',label:'SCD30 · CO₂, temperature & humidity',type:'toggle'},
    {key:'sensor_bme',label:'BME680 · climate, gas & BSEC IAQ',type:'toggle'},
    {key:'sensor_light',label:'TCS34725 · color & light',type:'toggle'},
    {key:'sensor_uv',label:'VEML6070 · ultraviolet',type:'toggle'}]},
  {title:'Sampling & history',description:'A fixed 600-point buffer uses 57.6 KB of RAM. Faster sampling reduces the available history.',fields:[
    {key:'history_interval_s',label:'History sampling',type:'select',options:[[0,'Automatic · 24 hours (every 2½ min)'],[5,'Every 5 seconds · about 50 minutes'],[15,'Every 15 seconds · about 2½ hours'],[30,'Every 30 seconds · about 5 hours'],[60,'Every minute · about 10 hours'],[150,'Every 2½ minutes · 24 hours'],[300,'Every 5 minutes · 24 hours'],[900,'Every 15 minutes · 24 hours'],[3600,'Every hour · 24 hours']],help:'Readings are held in device RAM and restart after power loss. Changing this interval starts a new history buffer.'},
    {key:'sensor_poll_ms',label:'SCD30 & UV poll interval (ms)',min:250,max:5000,step:1,help:'Polling reads available results; SCD30 has its own measurement interval.'},
    {key:'light_poll_ms',label:'Color sensor read interval (ms)',min:100,max:5000,step:1}]},
  {title:'SCD30',description:'Settings apply to the next measurement cycle. Pressure compensation takes priority over altitude.',fields:[
    {key:'scd_interval_s',label:'Measurement interval (seconds)',min:2,max:1800,step:1},
    {key:'scd_pressure_mbar',label:'Ambient pressure (mbar)',min:0,max:1400,step:1,help:'0 disables pressure compensation; otherwise use 700–1400 mbar.'},
    {key:'scd_temperature_offset',label:'Temperature offset (0.01 °C)',min:0,max:2000,step:1,help:'For example, 150 subtracts a 1.50 °C sensor heat offset.'},
    {key:'scd_altitude_m',label:'Altitude (metres)',min:0,max:3000,step:1},
    {key:'scd_asc',label:'Automatic self-calibration',type:'toggle',help:'Enable when the sensor regularly sees fresh outside air, as required for SCD30 self-calibration.'},
    {key:'scd_recovery',label:'Recover stalled measurements automatically',type:'toggle'}]},
  {title:'BME680 & BSEC',description:'Bosch BSEC controls the heater and 3-second measurement schedule. Accuracy is shown alongside the IAQ reading.',fields:[
    {key:'bsec_heat_offset',label:'Heat source offset (0.01 °C)',min:-2000,max:2000,step:1,help:'Applied to BSEC compensated temperature and humidity.'},
    {key:'bsec_save_interval_s',label:'Calibration save interval (seconds)',min:3600,max:86400,step:1,help:'Calibration also saves when accuracy improves. Default: 21600 seconds (6 hours).'}]},
  {title:'Color & ultraviolet',description:'Change gain and integration time to match the lighting around your station.',fields:[
    {key:'light_gain',label:'TCS34725 gain',type:'select',options:[[0,'1×'],[1,'4×'],[2,'16×'],[3,'60×']]},
    {key:'light_cycles',label:'TCS34725 integration cycles',min:1,max:256,step:1,help:'2.4 ms per cycle. Default 21 cycles ≈ 50 ms. Long exposures can saturate in bright light.'},
    {key:'uv_integration',label:'VEML6070 integration time',type:'select',options:[[0,'½T'],[1,'1T'],[2,'2T'],[3,'4T']],help:'Raw counts depend on integration time. The UV graph is not a calibrated UV index.'}]},
  {title:'PC receiver',description:'The direct USB network stays at 192.168.7.1; DHCP assigns your PC 192.168.7.2. These settings control the existing measurement stream.',fields:[
    {key:'receiver_enabled',label:'Send measurements to PC receiver',type:'toggle'},
    {key:'receiver_ip',label:'Receiver IPv4 address',type:'text',maxLength:15,pattern:'(?:[0-9]{1,3}\\.){3}[0-9]{1,3}',help:'Use 192.168.7.2 for the PC connected by USB.'},
    {key:'receiver_port',label:'Receiver TCP port',min:1,max:65535,step:1},
    {key:'send_interval_ms',label:'Send interval (ms)',min:250,max:3600000,step:1}]}
];
const sensorFields:Record<string,number>={sensor_scd:1,sensor_bme:2,sensor_light:16,sensor_uv:8};
function fieldHtml(field:Field){
  const type=field.type??'number',id=`field-${field.key}`,help=field.help?`<small id="help-${field.key}">${field.help}</small>`:'';
  const attributes=`id="${id}" name="${field.key}" ${field.help?`aria-describedby="help-${field.key}"`:''}`;
  if(type==='toggle')return `<label class="field toggle" for="${id}"><span><span class="field-label">${field.label}</span>${help}</span><input ${attributes} type="checkbox" role="switch"></label>`;
  const input=type==='select'?`<select ${attributes}>${field.options!.map(([v,t])=>`<option value="${v}">${t}</option>`).join('')}</select>`:
    `<input ${attributes} type="${type}" required ${field.min!==undefined?`min="${field.min}" max="${field.max}" step="${field.step??1}"`:''} ${field.pattern?`pattern="${field.pattern}"`:''} ${field.maxLength?`maxlength="${field.maxLength}"`:''} autocomplete="off">`;
  return `<label class="field" for="${id}"><span class="field-label">${field.label}</span>${input}${help}</label>`;
}
function renderSettings(){
  $('settings-fields').innerHTML=groups.map(g=>`<section class="settings-card"><h2>${g.title}</h2><p class="section-copy">${g.description}</p>${g.fields.map(fieldHtml).join('')}</section>`).join('');
  populateSettings();
  $<HTMLFormElement>('settings-form').addEventListener('input',()=>{dirty=true;$('save-label').textContent='You have unsaved changes';$('save-help').textContent='Save to apply changes and keep them through a restart.';});
  $<HTMLFormElement>('settings-form').addEventListener('submit',saveSettings);
}
function populateSettings(){
  if(!info)return;
  for(const group of groups)for(const field of group.fields){
    const input=$<HTMLInputElement|HTMLSelectElement>(`field-${field.key}`),value=field.key in sensorFields?!!(Number(info.config.sensor_mask)&sensorFields[field.key]):info.config[field.key];
    if(input instanceof HTMLInputElement&&input.type==='checkbox')input.checked=!!Number(value);
    else {
      if(input instanceof HTMLSelectElement && ![...input.options].some(option=>option.value===String(value)))
        input.add(new Option(`Custom · every ${value} seconds`,String(value)));
      input.value=String(value);
    }
    if(field.key in sensorFields)input.disabled=!(info.capabilities.sensor_mask&sensorFields[field.key]);
  }
}
function updateDeviceInfo(){
  if(!info)return;
  $('station-name').textContent=String(info.config.device_name);document.title=`${info.config.device_name} · air`;
  $('footer-device').textContent=`${info.config.hostname}.local · USB Ethernet`;
  const link=$<HTMLAnchorElement>('device-link');link.textContent=`${info.config.hostname}.local`;link.href=`http://${info.config.hostname}.local/`;
  const seconds=info.storage.interval_s;
  $('history-description').textContent=`Up to ${Math.min(24,info.storage.retention_s/3600).toLocaleString(undefined,{maximumFractionDigits:1})} hours · one recorded point every ${seconds<60?`${seconds} sec`:`${(seconds/60).toLocaleString(undefined,{maximumFractionDigits:1})} min`} · stored on your station`;
  const s=info.storage;
  $('memory-stats').className='memory-stats';
  $('memory-stats').innerHTML=[['Firmware flash',s.firmware_bytes,1008*1024],['Main RAM reserved',s.ram1_static_bytes,s.ram1_total_bytes],['History RAM reserved',s.history_bytes,s.ram2_total_bytes]].map(([label,used,total])=>`<div class="memory-stat">${label}<strong>${(Number(used)/1024).toFixed(1)} / ${(Number(total)/1024).toFixed(0)} KB</strong><div class="memory-meter"><span style="width:${Number(used)/Number(total)*100}%"></span></div></div>`).join('');
}
async function loadConfig(){
  const previousInterval=info?.config.history_interval_s;
  info=await(await request('/api/config')).json() as DeviceInfo;
  if(previousInterval!==undefined&&previousInterval!==info.config.history_interval_s){history=[];cursor=0;lastHistory=0;}
  updateDeviceInfo();
}
async function saveSettings(event:SubmitEvent){
  event.preventDefault();if(saving||!info)return;
  const form=$<HTMLFormElement>('settings-form');if(!form.reportValidity())return;
  const data=new FormData(form),params=new URLSearchParams();let mask=0;
  for(const group of groups)for(const field of group.fields){
    if(field.key in sensorFields){if(data.get(field.key))mask|=sensorFields[field.key];continue;}
    params.set(field.key,field.type==='toggle'?(data.has(field.key)?'1':'0'):String(data.get(field.key)));
  }
  params.set('sensor_mask',String(mask));
  const pressure=Number(params.get('scd_pressure_mbar'));
  if(pressure!==0&&(pressure<700||pressure>1400)){toast('Ambient pressure must be 0 or between 700 and 1400 mbar.',true);return;}
  const previousInterval=Number(info.config.history_interval_s);
  saving=true;$<HTMLButtonElement>('save').disabled=true;$('save').textContent='Saving…';
  try{
    info=await(await request('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Air-Config':'1'},body:params.toString()})).json() as DeviceInfo;
    dirty=false;updateDeviceInfo();populateSettings();
    $('save-label').textContent='Saved to device flash';$('save-help').textContent=`Settings record ${info.storage.sequence} · changes are now active.`;
    if(previousInterval!==Number(info.config.history_interval_s)){history=[];cursor=0;lastHistory=0;}
    toast('Settings saved. Your station is up to date.');
    if(location.hostname.endsWith('.local')&&location.hostname!==`${info.config.hostname}.local`){setTimeout(()=>location.href=`http://${info!.network.ip}/#settings`,1200);}
  }catch(error){toast(error instanceof Error?error.message:'Could not save settings',true);$('save-label').textContent='Settings have not been saved';}
  finally{saving=false;$<HTMLButtonElement>('save').disabled=false;$('save').textContent='Save to device';}
}
async function loadHistory(){
  // Small immutable pages bound device RAM and prevent a long transfer from
  // racing the history ring. At most 600 records are retained in this browser.
  for(let page=0;page<20;page++){
    const {rows}=decodeHistory(await(await request(`/api/history?after=${cursor}`)).arrayBuffer());
    if(rows.length===0)break;
    history.push(...rows);cursor=rows[rows.length-1].sequence;
    if(rows.length<32)break;
  }
  const cutoff=Date.now()-86400000;
  history=history.filter(r=>r.time>=cutoff).slice(-600);
  lastHistory=Date.now();
}
async function poll(){
  try{
    const next=await(await request('/api/measurements')).json() as Live;
    const rebooted=!!live&&next.uptime_ms<live.uptime_ms&&((next.uptime_ms-live.uptime_ms)>>>0)>30000;
    if(rebooted){history=[];cursor=0;lastHistory=0;}
    live=next;
    if(lastRevision!==next.config_revision){await loadConfig();if(!dirty)populateSettings();lastRevision=next.config_revision;}
    if(Date.now()-lastHistory>10000||next.history_count>history.length)await loadHistory();
    failed=0;$('offline-banner').hidden=true;$('connection').classList.remove('offline');$('connection').innerHTML='<i></i>Connected via USB';
    updateReadings();
  }catch{
    failed++;if(failed>1){$('offline-banner').hidden=false;$('connection').classList.add('offline');$('connection').innerHTML='<i></i>Reconnecting';$('last-update').textContent='Waiting for the device';}
  }finally{setTimeout(poll,failed?4000:2000);}
}
function navigate(){
  const settings=location.hash==='#settings',name=settings?'settings':location.hash==='#measurements'?'measurements':'overview';
  $('settings-page').hidden=!settings;$('overview-page').hidden=settings;$('page-name').textContent=settings?'Configuration':name==='measurements'?'Measurements':'Overview';
  qa<HTMLElement>('[data-nav]').forEach(e=>e.classList.toggle('active',e.dataset.nav===name));
  if(name==='measurements')requestAnimationFrame(()=>$('measurements').scrollIntoView({behavior:'smooth'}));else window.scrollTo({top:0});
}
function exportCsv(){
  const ordered=[...metrics].filter(enabledMetric).sort((a,b)=>a.column-b.column);
  const lines=[['Timestamp',...ordered.map(m=>`${m.title} (${m.unit})`)].join(','),...history.map(row=>[new Date(row.time).toISOString(),...ordered.map(m=>rowValue(row,m.column)??'')].join(','))];
  const blob=new Blob([lines.join('\r\n')],{type:'text/csv;charset=utf-8'}),url=URL.createObjectURL(blob),a=document.createElement('a');
  a.href=url;a.download=`air-history-${new Date().toISOString().slice(0,10)}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
async function start(){
  if(!initialized){
  initialized=true;
  $('today').textContent=new Intl.DateTimeFormat(undefined,{weekday:'short',day:'numeric',month:'short',year:'numeric'}).format(new Date());
  qa<HTMLButtonElement>('[data-hours]').forEach(b=>b.addEventListener('click',()=>{hours=Number(b.dataset.hours);qa('[data-hours]').forEach(e=>e.classList.toggle('selected',e===b));for(const column of pendingCharts.keys())drawMetric(column);}));
  qa<HTMLButtonElement>('button[data-group]').forEach(b=>b.addEventListener('click',()=>{category=b.dataset.group!;applyFilters();}));
  $('export').addEventListener('click',exportCsv);window.addEventListener('hashchange',navigate);
  window.addEventListener('beforeunload',event=>{if(dirty){event.preventDefault();event.returnValue='';}});
  }
  try{await loadConfig();renderMetrics();renderSettings();navigate();void poll();}
  catch(error){$('connection').textContent='Device unavailable';$('offline-banner').hidden=false;toast(error instanceof Error?error.message:'Could not connect',true);setTimeout(start,4000);}
}
void start();
