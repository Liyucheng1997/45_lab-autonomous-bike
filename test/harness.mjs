import { params } from '../src/core/params.js';
import { buildPlant, linearize } from '../src/core/plant.js';
import { initialState, actuate, step } from '../src/core/physics.js';
import { MotionController } from '../src/core/controller.js';
import { SensorSuite, Estimator } from '../src/core/estimator.js';
import { eigenvalues, mat } from '../src/core/mathx.js';

export function makeSim(over = {}) {
  const p = { ...params, ...over };
  const plant = buildPlant(p);
  const ctl = new MotionController(plant);
  const sens = new SensorSuite(p);
  const est = new Estimator(p);
  let s = initialState();
  let t = 0, fallen = false;
  let maxPhi = 0, maxOm = 0, maxTau = 0;
  const api = {
    p, plant, ctl, get s() { return s; }, set s(v) { s = v; }, get t() { return t; },
    get fallen() { return fallen; }, stats: () => ({ maxPhi, maxOm, maxTau }), resetStats() { maxPhi = maxOm = maxTau = 0; },
    run(T, hook) {
      const n = Math.round(T / p.dt);
      for (let i = 0; i < n && !fallen; i++) {
        hook && hook(api);
        let xh;
        if (p.useEstimator) { est.update(sens.read(s, plant), p.dt, plant); xh = est; }
        else xh = s;
        const cmd = ctl.update(xh, p.dt);
        const a = actuate(plant, s, cmd);
        s = step(plant, s, a, p.dt);
        t += p.dt;
        maxPhi = Math.max(maxPhi, Math.abs(s.phi)); maxOm = Math.max(maxOm, Math.abs(s.Omega)); maxTau = Math.max(maxTau, Math.abs(a.tau));
        if (Math.abs(s.phi) > p.maxFallAngle || !isFinite(s.phi)) fallen = true;
      }
      return api;
    },
  };
  return api;
}
export { linearize, eigenvalues, mat };
