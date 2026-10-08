import { makeSim, linearize, eigenvalues, mat } from './harness.mjs';
const sim = makeSim();
const { plant } = sim;
const b = plant.body;
console.log('total mass', b.totalMass.toFixed(2), 'kg  CoM', b.cg.map(v=>v.toFixed(3)), ' Iw', plant.Iw.toFixed(4));
console.log('whipple', Object.fromEntries(Object.entries(b.whipple).map(([k,v])=>[k,+v.toFixed(4)])));
console.log('M', plant.W.M.map(r=>r.map(v=>v.toFixed(4))));
console.log('C1', plant.W.C1.map(r=>r.map(v=>v.toFixed(4))));
console.log('K0', plant.W.K0.map(r=>r.map(v=>v.toFixed(4))));
console.log('K2', plant.W.K2.map(r=>r.map(v=>v.toFixed(4))));
// open-loop self stability of bare bike (4-state)
for (const v of [0,1,2,3,4,5,6,7,8]) {
  const {A,B} = linearize(plant, v);
  const ev = eigenvalues(A).map(e=>e.re).sort((a,b)=>b-a)[0];
  const tab = plant.gains.find(g=>Math.abs(g.v-v)<1e-9);
  const Acl = mat.sub(A, mat.mul(B, tab.K));
  const evc = eigenvalues(Acl).map(e=>e.re).sort((a,b)=>b-a)[0];
  console.log(`v=${v}  open max Re=${ev.toFixed(3)}  closed max Re=${evc.toFixed(3)}  K=`, tab.K.map(r=>r.map(k=>k.toFixed(1)).join(',')).join(' | '));
}
