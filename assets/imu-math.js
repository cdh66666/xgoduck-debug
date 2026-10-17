/* Pure transformations of telemetry. No offsets are applied to the robot. */
(function(root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.XgoImuMath = api;
})(typeof window === 'undefined' ? globalThis : window, function() {
  'use strict';
  const vector = v => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite);
  function attitude(acc) {
    if (!vector(acc)) return null;
    const [x,y,z] = acc, norm = Math.hypot(x,y,z), deg = 180 / Math.PI;
    if (norm < 0.1) return null;
    return {roll: Math.atan2(-y,-z)*deg, pitch: Math.atan2(x,Math.hypot(y,z))*deg,
      tilt: Math.atan2(Math.hypot(x,y),-z)*deg, norm, inverted: z > 0};
  }
  function describe(rows, key) {
    const values = rows.filter(row => vector(row[key])).map(row => row[key]);
    if (!values.length) return null;
    const mean = [0,1,2].map(i => values.reduce((sum,v) => sum+v[i],0)/values.length);
    return {count:values.length,mean,std:[0,1,2].map(i=>Math.sqrt(values.reduce((sum,v)=>sum+(v[i]-mean[i])**2,0)/values.length))};
  }
  function sequenceStep(previous, next, sameInstance) {
    if (!sameInstance || !Number.isInteger(previous)) return 'reset';
    const delta = (next-previous) >>> 0;
    return delta === 0 ? 'duplicate' : delta < 0x80000000 ? 'advance' : 'reset';
  }
  return {vector,attitude,describe,sequenceStep};
});
