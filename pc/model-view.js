import * as THREE from 'three';
import { OrbitControls } from './model-assets/vendor/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from './model-assets/vendor/examples/jsm/loaders/STLLoader.js';

const $ = id => document.getElementById(id);
const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#edf2f4');
scene.add(new THREE.HemisphereLight(0xffffff, 0x647985, 2.4));
const light = new THREE.DirectionalLight(0xffffff, 2.5);
light.position.set(.4, -.5, .8);
scene.add(light);
const camera = new THREE.PerspectiveCamera(39, 1, .001, 10);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor('#edf2f4');
viewport.append(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = .10;
controls.maxDistance = 1.2;
const grid = new THREE.GridHelper(.7, 28, 0x95acb6, 0xd6e1e6);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
const meshes = new Map();
const bases = new Map();
const axisGroup = new THREE.Group();
const markerGroup = new THREE.Group();
scene.add(axisGroup, markerGroup);
let data, selected, affected = new Set(), markersVisible = true, playing = false, phaseStart = 0;
let pendingId = Number(new URLSearchParams(location.search).get('id')) || 10;
let angle = 0;
let referencePose = 'q-zero';
const initialView = () => {
  camera.position.set(.38, -.43, .30);
  controls.target.set(.005, 0, .125);
  controls.update();
};
initialView();
function resize() {
  const { width, height } = viewport.getBoundingClientRect();
  camera.aspect = width / Math.max(height, 1);
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}
new ResizeObserver(resize).observe(viewport);
resize();
function labelSprite(text, color = '#236776') {
  const canvas = document.createElement('canvas');
  canvas.width = 96; canvas.height = 64;
  const context = canvas.getContext('2d');
  context.fillStyle = '#ffffffed'; context.fillRect(3, 5, 90, 54);
  context.lineWidth = 3; context.strokeStyle = color; context.strokeRect(3, 5, 90, 54);
  context.fillStyle = color; context.font = 'bold 36px sans-serif';
  context.textAlign = 'center'; context.textBaseline = 'middle'; context.fillText(text, 48, 33);
  const texture = new THREE.CanvasTexture(canvas);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false }));
  sprite.scale.set(.0135, .009, 1);
  return sprite;
}
function descendants(name) {
  const result = new Set([name]);
  for (const record of data.records) if (record.parent === name) for (const child of descendants(record.name)) result.add(child);
  return result;
}
function status() {
  $('status').textContent = `已加载 ${meshes.size} 个同版连杆 · ID ${selected.id} · ${referencePose === 'q-zero' ? 'q=0 坐标参考' : 'runtime 默认参考'}`;
  document.body.dataset.loaded = 'true';
  document.body.dataset.selectedId = String(selected.id);
  document.body.dataset.meshCount = String(meshes.size);
  document.body.dataset.referencePose = referencePose;
  document.body.dataset.referenceJointDeg = String(selected.angle_deg ?? 0);
}
function setAngle(degrees) {
  angle = selected?.axis ? degrees : 0;
  const delta = new THREE.Matrix4();
  if (selected?.axis) {
    const pivot = new THREE.Vector3(...selected.origin_m);
    delta.makeTranslation(...pivot.toArray());
    delta.multiply(new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...selected.axis).normalize(), THREE.MathUtils.degToRad(angle)));
    delta.multiply(new THREE.Matrix4().makeTranslation(...pivot.clone().negate().toArray()));
  }
  for (const [name, mesh] of meshes) {
    mesh.matrix.copy(bases.get(name));
    if (affected.has(name)) mesh.matrix.premultiply(delta);
    mesh.matrixWorldNeedsUpdate = true;
  }
  for (const marker of markerGroup.children) {
    marker.position.copy(marker.userData.basePosition);
    if (affected.has(marker.userData.name)) marker.position.applyMatrix4(delta);
  }
  $('angle').value = String(angle);
  $('angle-value').textContent = `${angle.toFixed(1)}°`;
  document.body.dataset.modelAngle = angle.toFixed(3);
}
function halt() {
  playing = false;
  $('animate').textContent = '仅模型演示 ±3°';
  $('animate').classList.remove('active');
}
function drawAxis() {
  for (const object of [...axisGroup.children]) {
    axisGroup.remove(object);
    object.geometry?.dispose(); object.material?.dispose();
  }
  if (!selected.axis) return;
  const axis = new THREE.Vector3(...selected.axis).normalize();
  const pivot = new THREE.Vector3(...selected.origin_m);
  const start = pivot.clone().addScaledVector(axis, -.018);
  axisGroup.add(new THREE.ArrowHelper(axis, start, .036, 0x127594, .007, .004));
  const dot = new THREE.Mesh(new THREE.SphereGeometry(.0032, 14, 10), new THREE.MeshBasicMaterial({ color: 0xf29122, depthTest: false }));
  dot.position.copy(pivot); dot.renderOrder = 3; axisGroup.add(dot);
}
function selectJoint(id, notify = false) {
  if (!data) { pendingId = id; return; }
  const joint = data.joints.find(record => record.id === id);
  if (!joint) return;
  halt(); selected = { ...joint, ...(data.poses?.[referencePose]?.joints[String(id)] || {}) };
  $('joint').value = String(id);
  affected = joint.axis ? descendants(joint.name) : new Set([joint.name]);
  for (const [name, mesh] of meshes) {
    const primary = name === joint.name;
    if (affected.has(name)) mesh.visible = true;
    mesh.material.color.setHex(primary ? 0xf3a448 : affected.has(name) && joint.axis ? 0xf0c383 : mesh.userData.baseColor);
    mesh.material.emissive.setHex(primary ? 0x48240a : 0x000000);
  }
  $('part').value = joint.name;
  const available = Boolean(joint.axis);
  $('animate').disabled = !available; $('zero').disabled = !available; $('angle').disabled = !available;
  $('description').textContent = `ID ${id} · ${joint.label}`;
  $('joint-note').textContent = available
    ? '橙色为当前连杆，淡橙色为随动子链。蓝色箭头为同版模型转轴；±3°仅教学偏角，不能当实物指令或已验证行程。'
    : 'ID34 的独立嘴部网格与转轴未包含在该模型。橙色仅是头部关联区域，不能据此确认嘴部位置、开合方向或安全行程。';
  const homeAngle = data.zero_reference?.runtime_default_deg_by_id[String(id)];
  $('pose-note').textContent = `当前关节：MJCF 坐标零 = 0°；runtime 默认角 = ${homeAngle ?? '未知'}°。整机默认站姿包含非零角偏置。模型无法证明你的实物零位，先对照装配定位与实际舵盘安装，不能只按投影图保存零点。`;
  const record = data.records.find(item => item.name === joint.name);
  const parent = data.records.find(item => item.name === record.parent);
  const chain = [...affected].map(name => data.records.find(item => item.name === name)?.label).join(' → ');
  $('chain-note').textContent = available ? `上游固定件：${parent?.label || '躯干'}；所选关节带动：${chain}` : '嘴部没有独立刚性连杆；runtime 默认嘴角 0° 仅是控制坐标定义。';
  drawAxis(); setAngle(0); status(); hiddenCount();
  if (notify && window.parent !== window) window.parent.postMessage({ type: 'model-joint-selected', id }, location.origin);
}
window.addEventListener('message', event => {
  if (event.origin !== location.origin || event.source !== window.parent) return;
  if (event.data?.type === 'select-joint' && Number.isInteger(event.data.id)) selectJoint(event.data.id);
});
function setReferencePose(value) {
  if (!data?.poses?.[value]) return;
  referencePose = value;
  $('pose').value = value;
  halt();
  const pose = data.poses[value];
  for (const [name, matrix] of Object.entries(pose.records)) {
    bases.set(name, new THREE.Matrix4().set(...matrix.flat()));
  }
  for (const marker of markerGroup.children) {
    const joint = pose.joints[String(marker.userData.id)];
    marker.userData.basePosition.fromArray(joint.origin_m).add(new THREE.Vector3(0, marker.userData.id < 20 ? .007 : marker.userData.id < 30 ? -.007 : 0, .007));
  }
  selectJoint(selected?.id || pendingId);
}
function hiddenCount() {
  const count = [...meshes.values()].filter(mesh => !mesh.visible).length;
  $('restore').textContent = `显示全部隐藏件（${count}）`;
  document.body.dataset.hiddenCount = String(count);
  for (const marker of markerGroup.children) marker.visible = meshes.get(marker.userData.name)?.visible !== false;
}
$('joint').onchange = event => selectJoint(Number(event.target.value), true);
$('pose').onchange = event => setReferencePose(event.target.value);
$('home').onclick = () => { halt(); setAngle(0); initialView(); };
$('markers').onclick = () => {
  markersVisible = !markersVisible; markerGroup.visible = markersVisible;
  $('markers').textContent = `关节标记：${markersVisible ? '开' : '关'}`;
  $('markers').classList.toggle('active', markersVisible);
};
$('animate').onclick = () => {
  if (!selected?.axis) return;
  playing = !playing; phaseStart = performance.now();
  $('animate').textContent = playing ? '停止模型演示' : '仅模型演示 ±3°';
  $('animate').classList.toggle('active', playing);
  if (!playing) setAngle(0);
};
$('zero').onclick = () => { halt(); setAngle(0); };
$('angle').oninput = event => { halt(); setAngle(Number(event.target.value)); };
$('hide').onclick = () => { const mesh = meshes.get($('part').value); if (mesh) mesh.visible = false; hiddenCount(); };
$('isolate').onclick = () => { const name = $('part').value; for (const [key, mesh] of meshes) mesh.visible = name === key; hiddenCount(); };
$('restore').onclick = () => { for (const mesh of meshes.values()) mesh.visible = true; hiddenCount(); };
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let pointerDown;
renderer.domElement.addEventListener('pointerdown', event => { pointerDown = [event.clientX, event.clientY]; });
renderer.domElement.addEventListener('pointerup', event => {
  if (!data || !pointerDown || Math.hypot(event.clientX - pointerDown[0], event.clientY - pointerDown[1]) > 4) return;
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.set((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
  const markers = markersVisible ? raycaster.intersectObjects(markerGroup.children, false) : [];
  if (markers.length) return selectJoint(markers[0].object.userData.id, true);
  const hit = raycaster.intersectObjects([...meshes.values()].filter(mesh => mesh.visible), false)[0];
  if (hit) {
    const joint = data.joints.find(record => record.name === hit.object.name && record.axis);
    if (joint) selectJoint(joint.id, true);
    $('part').value = hit.object.name;
  }
});
function animate(time) {
  requestAnimationFrame(animate);
  if (playing && selected?.axis) setAngle(Math.sin((time - phaseStart) / 900) * 3);
  controls.update(); renderer.render(scene, camera);
}
requestAnimationFrame(animate);
try {
  const response = await fetch('./model-assets/model-data.json');
  if (!response.ok) throw new Error(`模型清单 HTTP ${response.status}`);
  data = await response.json();
  const loader = new STLLoader();
  await Promise.all(data.records.map(async record => {
    const geometry = await loader.loadAsync(`./model-assets/${record.name}.stl`);
    geometry.computeVertexNormals();
    const baseColor = record.name.includes('ankle') ? 0xdcb654 : record.name.includes('head') ? 0xe0e8eb : record.name.startsWith('left') ? 0x6c9cae : record.name.startsWith('right') ? 0x6c9e8a : 0xc2cdd3;
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: baseColor, roughness: .82, metalness: .04 }));
    mesh.name = record.name; mesh.userData.baseColor = baseColor;
    mesh.matrixAutoUpdate = false;
    const rows = record.matrix;
    mesh.matrix.set(...rows.flat());
    mesh.matrix.elements[12] /= 100; mesh.matrix.elements[13] /= 100; mesh.matrix.elements[14] /= 100;
    meshes.set(record.name, mesh); bases.set(record.name, mesh.matrix.clone()); scene.add(mesh);
  }));
  for (const record of data.records) $('part').add(new Option(record.label, record.name));
  for (const joint of data.joints) {
    $('joint').add(new Option(`${joint.id} · ${joint.label}`, String(joint.id)));
    if (joint.axis) {
      const marker = labelSprite(String(joint.id));
      marker.position.fromArray(joint.origin_m).add(new THREE.Vector3(0, joint.id < 20 ? .007 : joint.id < 30 ? -.007 : 0, .007));
      marker.userData.id = joint.id; marker.userData.name = joint.name;
      marker.userData.basePosition = marker.position.clone(); markerGroup.add(marker);
    }
  }
  setReferencePose('q-zero'); hiddenCount();
  if (window.parent !== window) window.parent.postMessage({ type: 'model-ready', version: 'R1', meshes: meshes.size, axes: 14 }, location.origin);
} catch (error) {
  $('status').textContent = `模型未加载：${error.message}`; $('status').classList.add('error');
  document.body.dataset.loaded = 'false';
  console.error(error);
}
