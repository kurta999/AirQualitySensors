import { readFile } from 'node:fs/promises';
import { transform } from 'esbuild';
import test from 'node:test';
import assert from 'node:assert/strict';
const source=await readFile(new URL('../src/protocol.ts',import.meta.url),'utf8');
const {code}=await transform(source,{loader:'ts',format:'esm'});
const {decodeHistory,rowValue,ageMs}=await import('data:text/javascript;base64,'+Buffer.from(code).toString('base64'));
function fixture(){
  const bytes=new ArrayBuffer(128),v=new DataView(bytes);
  [0x31524941,1,96,21,1,10000,150,42].forEach((n,i)=>v.setUint32(i*4,n,true));
  v.setUint32(32,8000,true);v.setUint32(36,59,true);
  for(let i=0;i<21;i++)v.setFloat32(40+i*4,i+.5,true);
  v.setUint8(124,2);v.setUint8(125,1);v.setUint8(126,1);
  return bytes;
}
test('history decoding preserves field order, timestamps and BSEC status',()=>{
  const {rows,interval}=decodeHistory(fixture(),1000000);
  assert.equal(interval,150);assert.equal(rows.length,1);assert.equal(rows[0].sequence,42);
  assert.equal(rows[0].time,998000);assert.equal(rowValue(rows[0],0),.5);
  assert.equal(rowValue(rows[0],13),13.5);assert.equal(rowValue(rows[0],21),2);
  assert.equal(rowValue(rows[0],22),1);assert.equal(rowValue(rows[0],23),1);
  assert.equal(rowValue(rows[0],19),null); // absent particle sensor
});
test('truncated and incompatible records fail without rendering fabricated data',()=>{
  for(let i=0;i<128;i++)assert.throws(()=>decodeHistory(fixture().slice(0,i)));
  const data=fixture();new DataView(data).setUint32(8,100,true);assert.throws(()=>decodeHistory(data));
});
test('invalid sources, nonfinite values and millisecond wrap are handled',()=>{
  const {rows}=decodeHistory(fixture());rows[0].values[0]=NaN;assert.equal(rowValue(rows[0],0),null);
  rows[0].values[0]=null;assert.equal(rowValue(rows[0],0),null);
  rows[0].mask=0;assert.equal(rowValue(rows[0],21),null);
  assert.equal(ageMs(1000,0xfffffff0),1016);
});
