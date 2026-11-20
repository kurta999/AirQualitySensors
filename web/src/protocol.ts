export const columnSources = [0,0,0,1,1,1,1,3,4,4,4,4,4,5,5,5,5,5,5,2,2] as const;
export interface HistoryRow {
  tick: number; time: number; mask: number; sequence: number;
  values: (number|null)[]; accuracy: number; stabilized: number; run_in: number;
}
export function ageMs(now: number, then: number): number { return (now - then) >>> 0; }
export function decodeHistory(buffer: ArrayBuffer, wallTime = Date.now()): { rows: HistoryRow[]; interval: number } {
  if (buffer.byteLength < 32) throw new Error('Incomplete history header');
  const view = new DataView(buffer);
  if (view.getUint32(0,true) !== 0x31524941 || view.getUint32(4,true) !== 1 ||
      view.getUint32(8,true) !== 96 || view.getUint32(12,true) !== 21) throw new Error('Unsupported history format');
  const count = view.getUint32(16,true), now = view.getUint32(20,true), first = view.getUint32(28,true);
  if (count > 32 || buffer.byteLength !== 32 + count * 96) throw new Error('Incomplete history records');
  const rows: HistoryRow[] = [];
  for (let i = 0; i < count; i++) {
    const offset = 32 + i * 96, tick = view.getUint32(offset,true);
    const values = Array.from({ length: 21 }, (_, column) => view.getFloat32(offset + 8 + column * 4,true));
    rows.push({ tick, time: wallTime - ageMs(now,tick), mask: view.getUint32(offset + 4,true),
      values, sequence: (first + i) >>> 0, accuracy: view.getUint8(offset + 92),
      stabilized: view.getUint8(offset + 93), run_in: view.getUint8(offset + 94) });
  }
  return { rows, interval: view.getUint32(24,true) };
}
export function rowValue(row: HistoryRow, column: number): number | null {
  const source = column < 21 ? columnSources[column] : 5;
  if (!(row.mask & (1 << source))) return null;
  const value = column < 21 ? row.values[column] : column === 21 ? row.accuracy : column === 22 ? row.stabilized : row.run_in;
  return Number.isFinite(value) ? value : null;
}
