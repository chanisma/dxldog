// 3D 화면: MuJoCo 모델의 보이는 부품(CAD 메시·모터 상자)으로 DXL Dog 를 만들고, 서버가 보내는 바디 자세(frame)를 적용한다.
// 좌표계는 MuJoCo 와 같은 Z-up 을 그대로 쓴다. 색은 모델에 정해 둔 색(rgba)을 쓴다.
import * as THREE from "three";
import { OrbitControls } from "/web/vendor/OrbitControls.js";

const CAM_OFFSET = [0.42, -0.5, 0.24];     // 목표점에서 카메라까지 (m) — 로봇 길이 약 30 cm

export class Viewer {
  constructor(el) {
    this.el = el;
    this.follow = true;
    this.bodies = [];
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    el.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0x10141a, 2.5, 7);
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.005, 50);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.minDistance = 0.18;
    this.controls.maxDistance = 4;
    this.target = new THREE.Vector3(0, 0, 0.1);
    this.resetView();

    this._lights();
    this._floor();
    new ResizeObserver(() => this._resize()).observe(el);
    this._resize();
    this.renderer.setAnimationLoop(() => this._tick());
  }

  _lights() {
    this.scene.add(new THREE.HemisphereLight(0xcfe0ff, 0x1a1d22, 1.0));
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    const sc = key.shadow.camera;
    sc.left = -0.6; sc.right = 0.6; sc.top = 0.6; sc.bottom = -0.6; sc.near = 0.1; sc.far = 5;
    key.shadow.bias = -0.0003;
    key.shadow.radius = 3;
    this.key = key;
    this.scene.add(key, key.target);
    const rim = new THREE.DirectionalLight(0x7aa8ff, 0.7);
    rim.position.set(-3, 3, 2);
    this.scene.add(rim);
  }

  _floor() {
    const ground = new THREE.Mesh(
      new THREE.CircleGeometry(15, 64),
      new THREE.MeshStandardMaterial({ color: 0x161b22, roughness: 0.95, metalness: 0 }));
    ground.receiveShadow = true;
    this.scene.add(ground);
    const grid = new THREE.GridHelper(10, 200, 0x262e3a, 0x1d232c);      // 5 cm 눈금
    grid.rotation.x = Math.PI / 2;
    grid.position.z = 0.0005;
    this.scene.add(grid);
    const major = new THREE.GridHelper(10, 20, 0x3a4658, 0x2f3947);       // 50 cm 눈금
    major.rotation.x = Math.PI / 2;
    major.position.z = 0.001;
    major.material.opacity = 0.7;
    major.material.transparent = true;
    this.scene.add(major);
  }

  async load() {
    const scene = await (await fetch("/api/scene")).json();
    const meshCache = {};
    const loadMesh = async (mid) => {
      if (!meshCache[mid]) {
        meshCache[mid] = fetch(`/api/mesh/${mid}`).then((r) => r.arrayBuffer()).then((buf) => {
          const head = new Uint32Array(buf, 0, 2);
          const [nv, nf] = head;
          const verts = new Float32Array(buf, 8, nv * 3);
          const faces = new Uint32Array(buf, 8 + nv * 12, nf * 3);
          const indexed = new THREE.BufferGeometry();
          indexed.setAttribute("position", new THREE.BufferAttribute(verts, 3));
          indexed.setIndex(new THREE.BufferAttribute(faces, 1));
          const g = indexed.toNonIndexed();          // CAD 부품은 모서리가 각지므로 면마다 법선 (부드럽게 뭉개지지 않게)
          g.computeVertexNormals();
          return g;
        });
      }
      return meshCache[mid];
    };
    for (let i = 0; i < scene.nbody; i++) {
      const grp = new THREE.Group();
      this.scene.add(grp);
      this.bodies.push(grp);
    }
    const mats = {};
    const material = (rgba) => {
      const k = rgba.join(",");
      if (!mats[k]) {
        const c = new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace);
        const dark = rgba[0] + rgba[1] + rgba[2] < 1;              // 모터·발: 조금 반짝이게
        mats[k] = new THREE.MeshStandardMaterial({ color: c, metalness: dark ? 0.35 : 0.05, roughness: dark ? 0.45 : 0.6 });
      }
      return mats[k];
    };
    for (const g of scene.geoms) {
      let geom;
      if (g.type === "mesh") geom = await loadMesh(g.mesh);
      else if (g.type === "box") geom = new THREE.BoxGeometry(2 * g.size[0], 2 * g.size[1], 2 * g.size[2]);
      else geom = new THREE.SphereGeometry(g.radius, 24, 16);
      const mesh = new THREE.Mesh(geom, material(g.rgba));
      mesh.position.set(...g.pos);
      mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.bodies[g.body].add(mesh);
    }
  }

  // frame: p = [x,y,z]*nbody, q = [w,x,y,z]*nbody (MuJoCo xpos / xquat)
  applyFrame(p, q) {
    for (let i = 0; i < this.bodies.length; i++) {
      const b = this.bodies[i];
      b.position.set(p[3 * i], p[3 * i + 1], p[3 * i + 2]);
      b.quaternion.set(q[4 * i + 1], q[4 * i + 2], q[4 * i + 3], q[4 * i]);
    }
    if (this.bodies[1]) this.target.set(p[3], p[4], Math.max(0.06, p[5] * 0.7));
  }

  resetView() {
    const t = this.target;
    this.camera.position.set(t.x + CAM_OFFSET[0], t.y + CAM_OFFSET[1], t.z + CAM_OFFSET[2]);
    this.controls.target.copy(t);
  }

  _resize() {
    const w = this.el.clientWidth, h = this.el.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = w + "px";
    this.renderer.domElement.style.height = h + "px";
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  _tick() {
    if (this.follow) {
      // 로봇을 따라 카메라와 목표점을 함께 부드럽게 옮긴다 (사용자가 돌린 각도는 유지)
      const delta = this.target.clone().sub(this.controls.target).multiplyScalar(0.12);
      this.controls.target.add(delta);
      this.camera.position.add(delta);
    }
    this.key.position.set(this.controls.target.x + 1.2, this.controls.target.y - 1.0, 2);
    this.key.target.position.copy(this.controls.target);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
