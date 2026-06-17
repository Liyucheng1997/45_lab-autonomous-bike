// lil-gui 控制面板：实时调参，关键参数变化后重算 LQR 增益。
import GUI from 'lil-gui';

export function buildGUI(params, controller, actions) {
  const gui = new GUI({ title: '自动驾驶自行车 · 控制台' });

  const recompute = () => controller.recomputeGain();

  const fDrive = gui.addFolder('驾驶');
  fDrive.add(params, 'vTarget', -params.vMax, params.vMax, 0.1).name('目标速度 m/s').listen();
  fDrive.add(actions, 'kick').name('扰动 (踢一脚) [空格]');
  fDrive.add(actions, 'reset').name('复位 [R]');
  fDrive.add(actions, 'figureEight').name('走 8 字路径');
  fDrive.add(actions, 'clearPath').name('清除路径');

  const fBalance = gui.addFolder('动量轮 LQR');
  fBalance.add(params.Q, '0', 1, 5000, 1).name('Q θ (倾角)').onFinishChange(recompute);
  fBalance.add(params.Q, '1', 1, 500, 1).name('Q θ̇ (角速度)').onFinishChange(recompute);
  fBalance.add(params.Q, '2', 0, 5, 0.01).name('Q Ω (轮速)').onFinishChange(recompute);
  fBalance.add(params, 'R', 0.05, 10, 0.05).name('R (控制代价)').onFinishChange(recompute);
  fBalance.add(params, 'wheelTorqueMax', 1, 20, 0.5).name('力矩上限 N·m');
  fBalance.add(params, 'wheelSpeedMax', 100, 1000, 10).name('转速上限 rad/s');

  const fSteer = gui.addFolder('转向控制');
  fSteer.add(params, 'steerBalanceKp', 0, 4, 0.05).name('平衡 Kp');
  fSteer.add(params, 'steerBalanceKd', 0, 2, 0.01).name('平衡 Kd');
  fSteer.add(params, 'steerHeadingKp', 0, 3, 0.05).name('航向 Kp');
  fSteer.add(params, 'steerHeadingKd', 0, 3, 0.05).name('航向 Kd');
  fSteer.add(params, 'steerBlendSpeed', 0, 4, 0.1).name('转向介入速度');
  fSteer.add(params, 'steerMax', 0.1, 0.9, 0.01).name('最大转角 rad');

  const fPhys = gui.addFolder('物理参数');
  fPhys.add(params, 'm', 5, 40, 0.5).name('质量 kg').onFinishChange(recompute);
  fPhys.add(params, 'h', 0.3, 0.9, 0.01).name('质心高 m').onFinishChange(recompute);
  fPhys.add(params, 'Ip', 1, 15, 0.1).name('车身惯量').onFinishChange(recompute);
  fPhys.add(params, 'Iw', 0.01, 0.3, 0.005).name('动量轮惯量').onFinishChange(recompute);
  fPhys.add(params, 'b', 0.6, 1.6, 0.02).name('轴距 m');
  fPhys.close();

  return gui;
}
