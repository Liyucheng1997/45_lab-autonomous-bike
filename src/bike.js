// 参数化自行车三维模型（对应 XUAN 配置：车架 + 前后轮 + 转向 + 动量轮）。
// 坐标约定（模型局部）：前进 = +X，横向轴 = Z，竖直 = Y。
//   - 路面轮绕 Z 旋转滚动
//   - 转向绕 Y
//   - 车身侧倾(roll)绕 X
//   - 动量轮绕 X 旋转（其反作用力矩正好作用在 roll 轴上）
//
// 层级： root(平移+航向) → rollGroup(侧倾) → 各部件
// 若以后要替换为 XUAN 的真实网格，用 loadExternal() 把 .glb 挂到 rollGroup 即可。

import * as THREE from 'three';

const WHEEL_R = 0.34;

export class Bike {
  constructor(params) {
    this.p = params;
    this.root = new THREE.Group();
    this.rollGroup = new THREE.Group();
    this.root.add(this.rollGroup);
    this._build();
  }

  _mat(color, metalness = 0.6, roughness = 0.4) {
    return new THREE.MeshStandardMaterial({ color, metalness, roughness });
  }

  // 在 X-Y 平面内两点间放一根圆管（自行车在该平面内为侧视图）。
  _tube(group, x1, y1, x2, y2, r, mat) {
    const a = new THREE.Vector3(x1, y1, 0);
    const b = new THREE.Vector3(x2, y2, 0);
    const len = a.distanceTo(b);
    const tube = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 14), mat);
    tube.position.copy(a.clone().add(b).multiplyScalar(0.5));
    tube.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
    group.add(tube);
    return tube;
  }

  _wheel() {
    const g = new THREE.Group();
    // 轮胎（环面，法线沿 Z）
    const tire = new THREE.Mesh(
      new THREE.TorusGeometry(WHEEL_R, 0.035, 18, 56),
      this._mat(0x14171c, 0.15, 0.85)
    );
    g.add(tire);
    // 胎面纹路（细环）
    const tread = new THREE.Mesh(
      new THREE.TorusGeometry(WHEEL_R + 0.002, 0.022, 12, 56),
      this._mat(0x0c0e11, 0.1, 0.95)
    );
    g.add(tread);
    // 轮辋（金属）
    const rim = new THREE.Mesh(
      new THREE.TorusGeometry(WHEEL_R - 0.045, 0.016, 12, 56),
      this._mat(0xd9dde4, 0.95, 0.18)
    );
    g.add(rim);
    const rimInner = new THREE.Mesh(
      new THREE.TorusGeometry(WHEEL_R - 0.06, 0.006, 8, 56),
      this._mat(0xc2c7d0, 0.9, 0.25)
    );
    g.add(rimInner);
    // 轮毂
    const hub = new THREE.Mesh(
      new THREE.CylinderGeometry(0.026, 0.026, 0.09, 18),
      this._mat(0x9aa1ad, 0.95, 0.25)
    );
    hub.rotation.x = Math.PI / 2; // 轴沿 Z
    g.add(hub);
    const hubFlange = new THREE.Mesh(
      new THREE.CylinderGeometry(0.045, 0.045, 0.03, 18),
      this._mat(0x8891a0, 0.9, 0.3)
    );
    hubFlange.rotation.x = Math.PI / 2;
    g.add(hubFlange);
    // 辐条（细，X 形交叉感）
    const spokeMat = this._mat(0xe6e9ef, 0.95, 0.15);
    const N = 16;
    for (let i = 0; i < N; i++) {
      const a = (i / N) * Math.PI * 2;
      const sp = new THREE.Mesh(
        new THREE.CylinderGeometry(0.0028, 0.0028, WHEEL_R - 0.05, 5),
        spokeMat
      );
      // 从轮毂指向轮辋的单根辐条
      const rMid = (WHEEL_R - 0.05) / 2;
      sp.position.set(Math.cos(a) * rMid, Math.sin(a) * rMid, (i % 2 ? 0.018 : -0.018));
      sp.rotation.z = a + Math.PI / 2;
      g.add(sp);
    }
    return g;
  }

  _build() {
    const frameMat = this._mat(0x2bd1c4, 0.55, 0.35);   // 车架主色（青绿）
    const frameMat2 = this._mat(0x18b3a7, 0.55, 0.35);  // 深一档
    const dark = this._mat(0x33373f, 0.6, 0.45);
    const rubber = this._mat(0x1a1c20, 0.2, 0.85);
    const metal = this._mat(0xb8bdc7, 0.95, 0.22);

    const R = WHEEL_R;
    const b = this.p.b;
    // 侧视图关键节点 (x 前进, y 高)
    const rearHub = [-b / 2, R];
    const frontHub = [b / 2, R];
    const bb = [-0.06, 0.27];                 // 五通(中轴)
    const seatTop = [-0.17, 0.60];            // 座管顶
    const headBot = [b / 2 - 0.10, 0.50];     // 头管下
    const headTop = [b / 2 - 0.04, 0.70];     // 头管上

    // --- 后轮（固定，绕 Z 滚动）---
    this.rearWheel = this._wheel();
    this.rearWheel.position.set(rearHub[0], rearHub[1], 0);
    this.rollGroup.add(this.rearWheel);

    // --- 车架（双三角 diamond frame）---
    this._tube(this.rollGroup, ...bb, ...seatTop, 0.022, frameMat);        // 座管
    this._tube(this.rollGroup, ...seatTop, ...headTop, 0.024, frameMat);   // 上管
    this._tube(this.rollGroup, ...bb, ...headBot, 0.026, frameMat2);       // 下管
    this._tube(this.rollGroup, ...headBot, ...headTop, 0.026, dark);       // 头管
    // 后上叉 / 后下叉（左右各一，沿 Z 分开）
    for (const dz of [-0.06, 0.06]) {
      const sg = new THREE.Group();
      sg.position.z = dz;
      this._tube(sg, ...seatTop, ...rearHub, 0.013, frameMat);  // seat stay
      this._tube(sg, ...bb, ...rearHub, 0.015, frameMat2);      // chain stay
      this.rollGroup.add(sg);
    }

    // --- 传动：牙盘 + 曲柄 + 脚踏 ---
    const chainring = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.085, 0.006, 28), metal);
    chainring.rotation.x = Math.PI / 2;
    chainring.position.set(bb[0], bb[1], 0.05);
    this.rollGroup.add(chainring);
    const crankBox = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.14, 14), dark);
    crankBox.rotation.x = Math.PI / 2;
    crankBox.position.set(bb[0], bb[1], 0);
    this.rollGroup.add(crankBox);
    for (const s of [-1, 1]) {
      const crank = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.022, 0.012), dark);
      crank.position.set(bb[0] + s * 0.05, bb[1] - s * 0.04, s * 0.075);
      crank.rotation.z = s * 0.5;
      this.rollGroup.add(crank);
      const pedal = new THREE.Mesh(new THREE.BoxGeometry(0.10, 0.012, 0.05), rubber);
      pedal.position.set(bb[0] + s * 0.11, bb[1] - s * 0.085, s * 0.085);
      this.rollGroup.add(pedal);
    }
    // 链条（简化为两条细线 BB→后轮）
    const chainMat = this._mat(0x6b6f78, 0.85, 0.4);
    this._tube(this.rollGroup, bb[0], bb[1] + 0.085, rearHub[0], rearHub[1] + 0.03, 0.006, chainMat);
    this._tube(this.rollGroup, bb[0], bb[1] - 0.085, rearHub[0], rearHub[1] - 0.03, 0.006, chainMat);

    // --- 座管 + 座垫 ---
    this._tube(this.rollGroup, seatTop[0], seatTop[1], seatTop[0] - 0.02, seatTop[1] + 0.12, 0.016, metal);
    const saddle = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.045, 0.11), dark);
    saddle.geometry.translate(0.04, 0, 0);
    saddle.position.set(seatTop[0] - 0.04, seatTop[1] + 0.14, 0);
    saddle.rotation.z = -0.05;
    this.rollGroup.add(saddle);
    const saddleNose = new THREE.Mesh(new THREE.ConeGeometry(0.045, 0.14, 12), dark);
    saddleNose.rotation.z = -Math.PI / 2;
    saddleNose.position.set(seatTop[0] + 0.10, seatTop[1] + 0.13, 0);
    this.rollGroup.add(saddleNose);

    // --- 前轮 + 转向（steerGroup 绕 Y 转向）---
    this.steerGroup = new THREE.Group();
    this.steerGroup.position.set(frontHub[0], frontHub[1], 0); // 以前轴为转向枢轴(近似)
    this.frontWheel = this._wheel();
    this.steerGroup.add(this.frontWheel);
    // 前叉（左右两根，从前轴向上到头管，带轻微前倾）
    const forkTopY = headBot[1] - R; // 相对前轴的高度
    const forkTopX = headBot[0] - frontHub[0];
    for (const dz of [-0.05, 0.05]) {
      const fg = new THREE.Group();
      fg.position.z = dz;
      this._tube(fg, 0, 0, forkTopX, forkTopY, 0.013, dark);
      this.steerGroup.add(fg);
    }
    // 立管(steerer)到车把
    const stemBotX = forkTopX, stemBotY = forkTopY;
    const stemTopY = headTop[1] - R + 0.04;
    const stemTopX = forkTopX + 0.02;
    this._tube(this.steerGroup, stemBotX, stemBotY, stemTopX, stemTopY, 0.016, metal);
    // 车把横管（沿 Z）
    const bar = new THREE.Mesh(new THREE.CylinderGeometry(0.013, 0.013, 0.46, 12), dark);
    bar.rotation.x = Math.PI / 2;
    bar.position.set(stemTopX - 0.02, stemTopY, 0);
    this.steerGroup.add(bar);
    for (const s of [-1, 1]) {
      const grip = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.10, 12), rubber);
      grip.rotation.x = Math.PI / 2;
      grip.position.set(stemTopX - 0.02, stemTopY, s * 0.18);
      this.steerGroup.add(grip);
    }
    this.rollGroup.add(this.steerGroup);

    // --- 动量轮（reaction wheel）：金属飞轮，绕 X 轴旋转，挂在车架后三角中部 ---
    const rwX = -0.02, rwY = 0.43;
    this.reactionWheel = new THREE.Group();
    const flywheel = new THREE.Mesh(
      new THREE.CylinderGeometry(0.16, 0.16, 0.035, 36),
      this._mat(0xc9ccd2, 0.95, 0.2)
    );
    flywheel.rotation.z = Math.PI / 2; // 轴 Y→X
    this.reactionWheel.add(flywheel);
    // 外圈配重（深色）
    const rwRing = new THREE.Mesh(
      new THREE.TorusGeometry(0.155, 0.022, 12, 40),
      this._mat(0x3b82f6, 0.7, 0.35)
    );
    rwRing.rotation.y = Math.PI / 2;
    this.reactionWheel.add(rwRing);
    // 辐板让旋转可见
    for (let i = 0; i < 5; i++) {
      const a = (i / 5) * Math.PI * 2;
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.28, 0.006), this._mat(0x1d4ed8, 0.7, 0.3));
      arm.rotation.x = a;
      this.reactionWheel.add(arm);
    }
    this.reactionWheel.position.set(rwX, rwY, 0);
    this.rollGroup.add(this.reactionWheel);
    // 动量轮电机外壳
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.10, 20), dark);
    motor.rotation.z = Math.PI / 2;
    motor.position.set(rwX, rwY, -0.085);
    this.rollGroup.add(motor);
    // 支架把电机连到车架
    this._tube(this.rollGroup, bb[0], bb[1], rwX, rwY, 0.012, dark);
    this._tube(this.rollGroup, seatTop[0], seatTop[1] - 0.06, rwX, rwY, 0.012, dark);

    // --- IMU / 控制盒 ---
    const ctrlBox = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.055, 0.075), this._mat(0x2f3540, 0.4, 0.6));
    ctrlBox.position.set(bb[0] + 0.04, bb[1] + 0.10, 0);
    this.rollGroup.add(ctrlBox);
    const led = new THREE.Mesh(new THREE.SphereGeometry(0.008, 8, 8),
      new THREE.MeshStandardMaterial({ color: 0x00ff88, emissive: 0x00aa55, emissiveIntensity: 1.5 }));
    led.position.set(bb[0] + 0.095, bb[1] + 0.12, 0.03);
    this.rollGroup.add(led);
    // 电池
    const battery = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.06, 0.05), this._mat(0x222831, 0.3, 0.7));
    battery.position.set(-0.02, 0.40, 0);
    battery.visible = false; // 被动量轮挡住, 留作占位
  }

  // 由仿真状态更新可视姿态。
  update(s) {
    // 平移：sim(x,y) 地面 → three(x,z)；航向绕 Y。
    this.root.position.set(s.x, 0, s.y);
    this.root.rotation.y = -s.psi;
    // 侧倾绕前进轴(局部 X)
    this.rollGroup.rotation.x = s.theta;

    this.rearWheel.rotation.z = -s.rearSpin;
    this.frontWheel.rotation.z = -s.frontSpin;
    this.steerGroup.rotation.y = s.delta;
    this.reactionWheel.rotation.x = s.omega * 0.02; // 缩放仅为可视
  }

  // 可选：载入 XUAN 的真实网格（需先把 STEP/Fusion 转成 .glb）。
  async loadExternal(url) {
    const { GLTFLoader } = await import('three/examples/jsm/loaders/GLTFLoader.js');
    const loader = new GLTFLoader();
    const gltf = await loader.loadAsync(url);
    while (this.rollGroup.children.length) this.rollGroup.remove(this.rollGroup.children[0]);
    this.rollGroup.add(gltf.scene);
    return gltf;
  }
}
