// Browser viewer of NUSim (module::WebViewer): renders the streamed scene, replays recordings and operates the
// built-in referee.
// MuJoCo conventions are kept throughout: z is up, quaternions arrive as (w, x, y, z).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const RECORDING_MAGIC = 'RCSSMJR1';
const MAX_FRAMES = 45000;        // rolling live buffer (30 minutes at 25 frames per second)
const LIVE_DELAY = 0.08;         // seconds the live view stays behind the newest frame, for smooth interpolation
const TEAM_COLORS = [0x2f6fed, 0xe5484d, 0xf5a524, 0x8b5cf6];
// jersey colours in the GameController's numbering: blue, red, yellow, black, white, green, orange, purple, brown, gray
const GC_COLOURS = [0x2563eb, 0xdc2626, 0xfacc15, 0x1f2937, 0xf8fafc, 0x16a34a, 0xf97316, 0x9333ea, 0x92400e, 0x9ca3af];
const SUN_DIRECTION = new THREE.Vector3(-0.45, -0.55, 0.7).normalize();

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- renderer, lights, sky

const renderer = new THREE.WebGLRenderer({ canvas: $('view'), antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.95;

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0xcfdfee, 70, 420);

const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 2000);
camera.position.set(0, -16, 9);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.maxPolarAngle = Math.PI / 2 - 0.02;
controls.minDistance = 0.4;
controls.maxDistance = 120;
controls.target.set(0, 0, 0.4);

// Sky: a painted panorama (deep blue overhead, hazy horizon, sun glow), rotated so that its "up" is the z axis.
// It is the backdrop and also lights the scene, so robot shells pick up soft reflections of sky and ground.
function skyTexture() {
  const width = 2048, height = 1024;
  const canvas = document.createElement('canvas');
  canvas.width = width; canvas.height = height;
  const ctx = canvas.getContext('2d');

  const gradient = ctx.createLinearGradient(0, 0, 0, height);
  gradient.addColorStop(0.00, '#1f5fb4');
  gradient.addColorStop(0.28, '#4d8fd6');
  gradient.addColorStop(0.44, '#a9cdee');
  gradient.addColorStop(0.50, '#dbe9f4');
  gradient.addColorStop(0.505, '#8f9c8c');
  gradient.addColorStop(1.00, '#56634f');
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  // soft clouds, stretched along the horizon
  for (let i = 0; i < 46; i++) {
    const x = Math.random() * width, y = height * (0.16 + Math.random() * 0.27);
    const rx = 90 + Math.random() * 220, ry = 10 + Math.random() * 24;
    for (let k = 0; k < 5; k++) {
      const cx = x + (Math.random() - 0.5) * rx, cy = y + (Math.random() - 0.5) * ry;
      const puff = ctx.createRadialGradient(cx, cy, 0, cx, cy, rx * 0.6);
      puff.addColorStop(0, 'rgba(255, 255, 255, 0.20)');
      puff.addColorStop(1, 'rgba(255, 255, 255, 0)');
      ctx.fillStyle = puff;
      ctx.save(); ctx.translate(cx, cy); ctx.scale(1, ry / rx * 2.2); ctx.translate(-cx, -cy);
      ctx.fillRect(cx - rx, cy - rx, 2 * rx, 2 * rx);
      ctx.restore();
    }
  }

  // sun: placed where the directional light comes from
  const azimuth = Math.atan2(SUN_DIRECTION.y, SUN_DIRECTION.x), elevation = Math.asin(SUN_DIRECTION.z);
  const sx = ((0.5 - azimuth / (2 * Math.PI)) % 1 + 1) % 1 * width, sy = (0.5 - elevation / Math.PI) * height;
  for (const offset of [-width, 0, width]) {
    const glow = ctx.createRadialGradient(sx + offset, sy, 0, sx + offset, sy, 260);
    glow.addColorStop(0, 'rgba(255, 252, 235, 1)');
    glow.addColorStop(0.05, 'rgba(255, 246, 214, 0.85)');
    glow.addColorStop(0.25, 'rgba(255, 240, 200, 0.22)');
    glow.addColorStop(1, 'rgba(255, 240, 200, 0)');
    ctx.fillStyle = glow;
    ctx.fillRect(sx + offset - 260, sy - 260, 520, 520);
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.mapping = THREE.EquirectangularReflectionMapping;
  return texture;
}

{
  const texture = skyTexture();
  scene.background = texture;
  scene.backgroundRotation.x = Math.PI / 2;
  scene.environment = texture;
  scene.environmentRotation.x = Math.PI / 2;
  scene.environmentIntensity = 0.7;
}

const skyLight = new THREE.HemisphereLight(0xcfe3ff, 0x4f6b45, 0.45);
scene.add(skyLight);

const sun = new THREE.DirectionalLight(0xfff1d6, 3.0);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
sun.shadow.bias = -0.0003;
sun.shadow.normalBias = 0.02;
sun.shadow.radius = 2.5;
scene.add(sun, sun.target);

let shadowReach = 0;
function fitSunToField(halfX, halfY) {
  fitSun(Math.hypot(halfX, halfY) + 2);
}

function fitSun(reach) {
  shadowReach = reach;
  sun.position.copy(SUN_DIRECTION).multiplyScalar(reach * 2.5);
  sun.target.position.set(0, 0, 0);
  const cam = sun.shadow.camera;
  cam.left = -reach; cam.right = reach; cam.top = reach; cam.bottom = -reach;
  cam.near = 1; cam.far = reach * 5;
  cam.updateProjectionMatrix();
}
fitSunToField(12, 8);

// surrounding ground, so the pitch does not float in the void (textured once the grass texture exists)
const surround = new THREE.Mesh(new THREE.CircleGeometry(700, 96), new THREE.MeshStandardMaterial({ color: 0x6f8a62, roughness: 1, metalness: 0, envMapIntensity: 0.15 }));
surround.position.z = -0.02;
surround.receiveShadow = true;
scene.add(surround);

// ---------------------------------------------------------------- procedural textures

function grassTexture(base, stripe) {
  const size = 512;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');

  // two mowing stripes per tile
  ctx.fillStyle = base; ctx.fillRect(0, 0, size / 2, size);
  ctx.fillStyle = stripe; ctx.fillRect(size / 2, 0, size / 2, size);

  // blades: many short, slightly tinted strokes
  for (let i = 0; i < 26000; i++) {
    const x = Math.random() * size, y = Math.random() * size;
    const light = Math.random() < 0.5;
    ctx.strokeStyle = light ? `rgba(190, 230, 140, ${0.05 + Math.random() * 0.10})` : `rgba(10, 40, 12, ${0.05 + Math.random() * 0.12})`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + (Math.random() - 0.5) * 3, y + 2 + Math.random() * 4);
    ctx.stroke();
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return texture;
}

function ballTexture() {
  // classic ball: dark patches around the 12 vertices of an icosahedron
  const phi = (1 + Math.sqrt(5)) / 2;
  const spots = [];
  for (const a of [-1, 1]) for (const b of [-phi, phi]) spots.push([0, a, b], [a, b, 0], [b, 0, a]);
  const norm = Math.hypot(1, phi);

  const width = 512, height = 256;
  const canvas = document.createElement('canvas');
  canvas.width = width; canvas.height = height;
  const ctx = canvas.getContext('2d');
  const image = ctx.createImageData(width, height);

  for (let v = 0; v < height; v++) {
    const polar = (v + 0.5) / height * Math.PI;
    for (let u = 0; u < width; u++) {
      const azimuth = (u + 0.5) / width * 2 * Math.PI;
      const dx = Math.sin(polar) * Math.cos(azimuth), dy = Math.cos(polar), dz = Math.sin(polar) * Math.sin(azimuth);
      let best = -1;
      for (const s of spots) best = Math.max(best, (dx * s[0] + dy * s[1] + dz * s[2]) / norm);
      const shade = best > 0.925 ? 26 : best > 0.915 ? 120 : 244;
      const i = (v * width + u) * 4;
      image.data[i] = image.data[i + 1] = image.data[i + 2] = shade;
      image.data[i + 3] = 255;
    }
  }
  ctx.putImageData(image, 0, 0);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

const pitchTexture = grassTexture('#3c8c38', '#48a040');
const turfTexture = grassTexture('#2f7d3c', '#2f7d3c');
{
  const apron = grassTexture('#5f9455', '#5f9455');
  apron.repeat.set(350, 350);
  surround.material.map = apron;
  surround.material.color.set(0xffffff);
}

// Goal net: a white grid, see-through between the cords.
function netTexture() {
  const size = 256;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, size, size);
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.95)';
  ctx.lineWidth = 5;
  for (let i = 0; i <= 4; i++) {
    const at = i * size / 4;
    ctx.beginPath(); ctx.moveTo(at, 0); ctx.lineTo(at, size); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, at); ctx.lineTo(size, at); ctx.stroke();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(5, 5);
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return texture;
}
const netMap = netTexture();

// Perimeter boards: plain colour panels (no real sponsors), tiled along the board.
function boardTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 1024; canvas.height = 64;
  const ctx = canvas.getContext('2d');
  const colors = ['#1d4ed8', '#f8fafc', '#0f766e', '#f59e0b', '#f8fafc', '#b91c1c', '#111827', '#f8fafc'];
  colors.forEach((color, i) => {
    ctx.fillStyle = color;
    ctx.fillRect(i * 128, 0, 128, 64);
    ctx.fillStyle = color === '#f8fafc' ? 'rgba(15, 23, 42, 0.85)' : 'rgba(255, 255, 255, 0.9)';
    // abstract mark: a ball and three bars
    ctx.beginPath(); ctx.arc(i * 128 + 28, 32, 11, 0, 2 * Math.PI); ctx.fill();
    for (let bar = 0; bar < 3; bar++) ctx.fillRect(i * 128 + 50, 18 + bar * 11, 58 - bar * 14, 6);
  });
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = THREE.RepeatWrapping;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return texture;
}
const boardMap = boardTexture();

// Stadium scoreboard: an LED style panel above the stand, redrawn whenever the game state changes.
const scoreCanvas = document.createElement('canvas');
scoreCanvas.width = 1024; scoreCanvas.height = 288;
const scoreMap = new THREE.CanvasTexture(scoreCanvas);
scoreMap.colorSpace = THREE.SRGBColorSpace;
scoreMap.anisotropy = renderer.capabilities.getMaxAnisotropy();
scoreMap.wrapS = THREE.RepeatWrapping;

function fitText(ctx, text, maxWidth, size, weight = 700) {
  do { ctx.font = `${weight} ${size}px system-ui, -apple-system, "Segoe UI", sans-serif`; size -= 2; } while (ctx.measureText(text).width > maxWidth && size > 12);
}

// Team colours: the GameController's jersey colours when it runs the game, else the viewer's defaults.
function teamColour(game, side) {
  const index = game ? (side === 0 ? game.colourLeft : game.colourRight) : undefined;
  return index === undefined ? TEAM_COLORS[side] : GC_COLOURS[index % GC_COLOURS.length];
}
const cssColour = (hex) => `#${hex.toString(16).padStart(6, '0')}`;

function drawScoreboard(game) {
  const ctx = scoreCanvas.getContext('2d');
  const w = scoreCanvas.width, h = scoreCanvas.height;
  ctx.fillStyle = '#0a0d12'; ctx.fillRect(0, 0, w, h);
  ctx.fillStyle = cssColour(teamColour(game, 0)); ctx.fillRect(0, 0, 18, h);
  ctx.fillStyle = cssColour(teamColour(game, 1)); ctx.fillRect(w - 18, 0, 18, h);
  ctx.fillStyle = '#151a22'; ctx.fillRect(18, h - 64, w - 36, 64);

  ctx.textBaseline = 'middle';
  ctx.fillStyle = '#f2f4f7';
  ctx.textAlign = 'center';
  fitText(ctx, game ? String(game.left) : 'HOME', 300, 58);
  ctx.fillText(game ? String(game.left) : 'HOME', 190, 112);
  fitText(ctx, game ? String(game.right) : 'AWAY', 300, 58);
  ctx.fillText(game ? String(game.right) : 'AWAY', w - 190, 112);

  ctx.fillStyle = '#ffd34d';
  ctx.font = '800 150px system-ui, -apple-system, "Segoe UI", sans-serif';
  ctx.fillText(`${game ? game.scoreLeft : 0} : ${game ? game.scoreRight : 0}`, w / 2, 116);

  ctx.fillStyle = '#7ee787';
  ctx.font = '700 44px ui-monospace, "SF Mono", Menlo, monospace';
  ctx.textAlign = 'left';
  ctx.fillText(game ? formatTime(game.time, false) : '--:--', 44, h - 30);
  ctx.fillStyle = '#c9d1d9';
  ctx.textAlign = 'right';
  const mode = game ? modeText(game.mode) : 'waiting for game state';
  fitText(ctx, mode.toUpperCase(), w - 280, 34, 600);
  ctx.fillText(mode.toUpperCase(), w - 44, h - 30);

  scoreMap.needsUpdate = true;
}

// ---------------------------------------------------------------- venue: UNSW Village Green
// An impression of the Village Green at UNSW Sydney, scaled around the robot pitch: uniform synthetic turf with
// multi-sport markings, the blue running track around it, the white roofed grandstand shelter, tall light poles,
// ball-stop fences behind the goals, the pink and blue courts next door, and trees and campus buildings beyond.

function villageTurfTexture(width, height) {
  const scale = 24; // pixels per metre
  const canvas = document.createElement('canvas');
  canvas.width = Math.round(width * scale); canvas.height = Math.round(height * scale);
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#2f7d3c'; ctx.fillRect(0, 0, canvas.width, canvas.height);
  for (let i = 0; i < 60000; i++) {
    ctx.fillStyle = Math.random() < 0.5 ? 'rgba(120, 190, 110, 0.10)' : 'rgba(8, 40, 16, 0.12)';
    ctx.fillRect(Math.random() * canvas.width, Math.random() * canvas.height, 2, 2);
  }
  const cx = canvas.width / 2, cy = canvas.height / 2;
  const X = (m) => cx + m * scale, Y = (m) => cy - m * scale;

  // the other sports' markings: long yellow lines, an orange boundary and rows of yellow crosses
  ctx.lineWidth = 0.09 * scale;
  ctx.strokeStyle = '#e8a93a';
  ctx.strokeRect(X(-width / 2 + 0.8), Y(height / 2 - 0.8), (width - 1.6) * scale, (height - 1.6) * scale);
  ctx.strokeStyle = '#f2cf4a';
  for (const y of [-height / 2 + 2.2, height / 2 - 2.2]) { ctx.beginPath(); ctx.moveTo(X(-width / 2 + 0.8), Y(y)); ctx.lineTo(X(width / 2 - 0.8), Y(y)); ctx.stroke(); }
  for (const x of [-width / 2 + 2.6, width / 2 - 2.6]) { ctx.beginPath(); ctx.moveTo(X(x), Y(-height / 2 + 0.8)); ctx.lineTo(X(x), Y(height / 2 - 0.8)); ctx.stroke(); }
  const cross = (x, y) => { ctx.beginPath(); ctx.moveTo(X(x - 0.3), Y(y)); ctx.lineTo(X(x + 0.3), Y(y)); ctx.moveTo(X(x), Y(y - 0.3)); ctx.lineTo(X(x), Y(y + 0.3)); ctx.stroke(); };
  for (let x = -width / 2 + 2.6; x <= width / 2 - 2.5; x += 2.6) for (const y of [-height / 2 + 1.5, height / 2 - 1.5]) cross(x, y);
  for (let y = -height / 2 + 3.4; y <= height / 2 - 3.3; y += 2.4) for (const x of [-width / 2 + 1.7, width / 2 - 1.7]) cross(x, y);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return texture;
}

function windowTexture(wall, glass) {
  const canvas = document.createElement('canvas');
  canvas.width = 64; canvas.height = 64;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = wall; ctx.fillRect(0, 0, 64, 64);
  ctx.fillStyle = glass; ctx.fillRect(10, 14, 18, 30); ctx.fillRect(38, 14, 18, 30);
  ctx.fillStyle = 'rgba(255, 255, 255, 0.25)'; ctx.fillRect(10, 14, 18, 4); ctx.fillRect(38, 14, 18, 4);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  return texture;
}

function roundedRect(shape, hx, hy, radius) {
  shape.moveTo(-hx + radius, -hy);
  shape.lineTo(hx - radius, -hy); shape.absarc(hx - radius, -hy + radius, radius, -Math.PI / 2, 0, false);
  shape.lineTo(hx, hy - radius); shape.absarc(hx - radius, hy - radius, radius, 0, Math.PI / 2, false);
  shape.lineTo(-hx + radius, hy); shape.absarc(-hx + radius, hy - radius, radius, Math.PI / 2, Math.PI, false);
  shape.lineTo(-hx, -hy + radius); shape.absarc(-hx + radius, -hy + radius, radius, Math.PI, 1.5 * Math.PI, false);
  return shape;
}

// Deterministic pseudo random numbers, so the venue looks the same on every load and in every replay.
function seededRandom(seed) {
  return () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
}

function buildVillageGreen(halfX, halfY) {
  const group = new THREE.Group();
  const random = seededRandom(2033);
  const flat = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.9, metalness: 0, envMapIntensity: 0.25, ...extra });

  // synthetic turf apron with the multi-sport markings
  const turfX = halfX + 5, turfY = halfY + 4.5;
  const turf = new THREE.Mesh(new THREE.PlaneGeometry(2 * turfX, 2 * turfY), new THREE.MeshStandardMaterial({ map: villageTurfTexture(2 * turfX, 2 * turfY), roughness: 0.95, metalness: 0, envMapIntensity: 0.2 }));
  turf.position.z = -0.006;
  turf.receiveShadow = true;
  group.add(turf);

  // blue running track around the turf, with white lane lines
  const trackWidth = 3.4, corner = 5;
  const outerX = turfX + trackWidth, outerY = turfY + trackWidth;
  const trackShape = roundedRect(new THREE.Shape(), outerX, outerY, corner + trackWidth);
  trackShape.holes.push(roundedRect(new THREE.Path(), turfX - 0.02, turfY - 0.02, corner));
  const track = new THREE.Mesh(new THREE.ShapeGeometry(trackShape, 24), flat(0x2f7fd4, { roughness: 0.8 }));
  track.position.z = -0.004;
  track.receiveShadow = true;
  group.add(track);
  const laneMaterial = new THREE.LineBasicMaterial({ color: 0xeaf2fb });
  for (let lane = 0; lane <= 3; lane++) {
    const offset = 0.25 + lane * (trackWidth - 0.5) / 3;
    const points = roundedRect(new THREE.Shape(), turfX + offset, turfY + offset, corner + offset).getPoints(24);
    const line = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(points), laneMaterial);
    line.position.z = 0.004;
    group.add(line);
  }

  // paved surround under everything else
  const paving = new THREE.Mesh(new THREE.ShapeGeometry(roundedRect(new THREE.Shape(), outerX + 16, outerY + 5, 6), 12), flat(0xb9b7ae));
  paving.position.z = -0.012;
  paving.receiveShadow = true;
  group.add(paving);

  // ball-stop fences: tall black netting behind both goals, a low rail along the near side
  const steelBlack = new THREE.MeshStandardMaterial({ color: 0x15181c, roughness: 0.5, metalness: 0.6 });
  const mesh = netMap.clone(); mesh.needsUpdate = true;
  const fence = (x1, y1, x2, y2, height, withNet) => {
    const length = Math.hypot(x2 - x1, y2 - y1), angle = Math.atan2(y2 - y1, x2 - x1);
    const posts = Math.max(2, Math.round(length / 3) + 1);
    for (let i = 0; i < posts; i++) {
      const t = i / (posts - 1);
      const post = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, height, 8), steelBlack);
      post.rotation.x = Math.PI / 2;
      post.position.set(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, height / 2);
      post.castShadow = true;
      group.add(post);
    }
    const rail = new THREE.Mesh(new THREE.BoxGeometry(length, 0.05, 0.05), steelBlack);
    rail.position.set((x1 + x2) / 2, (y1 + y2) / 2, height);
    rail.rotation.z = angle;
    group.add(rail);
    if (withNet) {
      const map = mesh.clone(); map.needsUpdate = true; map.repeat.set(length * 2.5, height * 2.5);
      const net = new THREE.Mesh(new THREE.PlaneGeometry(length, height), new THREE.MeshBasicMaterial({ map, color: 0x0c0e11, transparent: true, alphaTest: 0.35, side: THREE.DoubleSide }));
      net.position.set((x1 + x2) / 2, (y1 + y2) / 2, height / 2);
      // stand the panel upright, then turn it along the fence line
      net.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), angle).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2));
      group.add(net);
    }
  };
  const fx = turfX - 0.3, fy = turfY - 0.3;
  fence(-fx, -fy * 0.8, -fx, fy * 0.8, 4.5, true);
  fence(fx, -fy * 0.8, fx, fy * 0.8, 4.5, true);
  fence(-fx, -fy, fx, -fy, 1.1, false);

  // the white roofed grandstand shelter along the far side: three bays of tiered seating
  const white = new THREE.MeshStandardMaterial({ color: 0xf4f5f2, roughness: 0.55, metalness: 0.05 });
  const concrete = flat(0xa7adb3);
  const bays = 3, bayLength = Math.min(7.5, (2 * halfX) / bays - 0.6), shelterY = halfY + 1.6;
  const people = new THREE.InstancedMesh(new THREE.CapsuleGeometry(0.13, 0.26, 3, 8), new THREE.MeshStandardMaterial({ roughness: 0.85 }), 400);
  const heads = new THREE.InstancedMesh(new THREE.SphereGeometry(0.09, 10, 8), new THREE.MeshStandardMaterial({ roughness: 0.7 }), 400);
  const dummy = new THREE.Object3D(), tint = new THREE.Color();
  const shirts = [0xffd100, 0xffd100, 0x1c1c1c, 0xf8fafc, 0x2f6fed, 0xe5484d, 0x10b981]; // plenty of UNSW yellow
  const skin = [0xf1c9a5, 0xd9a47a, 0xa8714a, 0x6f4a32];
  let seated = 0;
  for (let bay = 0; bay < bays; bay++) {
    const bx = (bay - (bays - 1) / 2) * (bayLength + 0.8);
    for (let row = 0; row < 3; row++) {
      const step = new THREE.Mesh(new THREE.BoxGeometry(bayLength, 0.8, 0.4 * (row + 1)), concrete);
      step.position.set(bx, shelterY + row * 0.8, 0.2 * (row + 1));
      step.castShadow = step.receiveShadow = true;
      group.add(step);
      for (let x = -bayLength / 2 + 0.4; x < bayLength / 2 - 0.3; x += 0.6) {
        if (random() < 0.45) continue;
        const px = bx + x + (random() - 0.5) * 0.12, py = shelterY + row * 0.8, base = 0.4 * (row + 1);
        dummy.rotation.set(Math.PI / 2, 0, 0);
        dummy.position.set(px, py, base + 0.26); dummy.updateMatrix();
        people.setMatrixAt(seated, dummy.matrix); people.setColorAt(seated, tint.setHex(shirts[Math.floor(random() * shirts.length)]));
        dummy.position.set(px, py - 0.02, base + 0.62); dummy.updateMatrix();
        heads.setMatrixAt(seated, dummy.matrix); heads.setColorAt(seated, tint.setHex(skin[Math.floor(random() * skin.length)]));
        seated++;
      }
    }
    // white roof, tilted up towards the pitch, on slim columns
    const roof = new THREE.Mesh(new THREE.BoxGeometry(bayLength + 0.6, 4.6, 0.14), white);
    roof.position.set(bx, shelterY + 0.6, 3.5);
    roof.rotation.x = -0.14;
    roof.castShadow = roof.receiveShadow = true;
    group.add(roof);
    for (const side of [-1, 1]) {
      const column = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 3.3, 10), white);
      column.rotation.x = Math.PI / 2;
      column.position.set(bx + side * (bayLength / 2 - 0.2), shelterY + 2.1, 1.65);
      column.castShadow = true;
      group.add(column);
    }
  }
  people.count = heads.count = seated;
  people.castShadow = heads.castShadow = true;
  group.add(people, heads);

  // scoreboard screen at the end of the ground, like the big screen behind the far goal
  {
    const boardWidth = 6.4, boardHeight = boardWidth * scoreCanvas.height / scoreCanvas.width;
    const screen = new THREE.MeshStandardMaterial({ map: scoreMap, emissive: 0xffffff, emissiveMap: scoreMap, emissiveIntensity: 0.85, roughness: 0.4 });
    // box faces: +x, -x, +y, -y, +z, -z; after the turn below the -y face looks down the pitch
    const board = new THREE.Mesh(new THREE.BoxGeometry(boardWidth, 0.3, boardHeight), [steelBlack, steelBlack, steelBlack, screen, steelBlack, steelBlack]);
    board.position.set(outerX + 1.2, turfY * 0.45, 3.2 + boardHeight / 2);
    board.rotation.z = -Math.PI / 2 - 0.35;
    board.castShadow = true;
    group.add(board);
    for (const side of [-1, 1]) {
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 3.3, 10), steelBlack);
      leg.rotation.x = Math.PI / 2;
      leg.position.set(board.position.x - side * Math.sin(0.35) * 2, board.position.y - side * Math.cos(0.35) * 2, 1.65);
      leg.castShadow = true;
      group.add(leg);
    }
  }

  // light poles: tall and slim, one at each corner and one mid-way along the far side
  const lamp = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xfff4d6, emissiveIntensity: 2.0 });
  for (const [px, py] of [[-1, -1], [1, -1], [-1, 1], [1, 1], [0, 1]]) { // none mid-way on the near side: it would stand in front of the broadcast camera
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.16, 15, 10), steelBlack);
    pole.rotation.x = Math.PI / 2;
    pole.position.set(px * (turfX + 0.6), py * (turfY + 0.6), 7.5);
    pole.castShadow = true;
    const head = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.25, 0.7), lamp);
    head.position.set(pole.position.x, pole.position.y, 15.2);
    head.lookAt(0, 0, 0);
    group.add(pole, head);
  }

  // the pink courts on their blue pads, at both ends
  const court = (cx, cy, w, h) => {
    const pad = new THREE.Mesh(new THREE.PlaneGeometry(w + 3, h + 3), flat(0x2a78cf, { roughness: 0.8 }));
    pad.position.set(cx, cy, -0.008);
    const surface = new THREE.Mesh(new THREE.PlaneGeometry(w, h), flat(0xe79ab4, { roughness: 0.8 }));
    surface.position.set(cx, cy, -0.002);
    pad.receiveShadow = surface.receiveShadow = true;
    const hw = w / 2, hh = h / 2;
    const outline = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh], [-hw, -hh], [0, -hh], [0, hh]].map(([x, y]) => new THREE.Vector3(cx + x, cy + y, 0.004));
    const lines = new THREE.Line(new THREE.BufferGeometry().setFromPoints(outline), laneMaterial);
    const circle = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(new THREE.Path().absarc(cx, cy, Math.min(hw, hh) * 0.3, 0, 2 * Math.PI).getPoints(32).map((v) => new THREE.Vector3(v.x, v.y, 0.004))), laneMaterial);
    group.add(pad, surface, lines, circle);
  };
  court(outerX + 8.5, -outerY * 0.35, 9, 12);
  court(outerX + 8.5, outerY * 0.62, 9, 6);
  court(-(outerX + 8.5), outerY * 0.3, 9, 11);

  // trees around the ground
  const treeCount = 90;
  const trunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.22, 0.3, 1, 8), flat(0x5b4636), treeCount);
  const canopies = new THREE.InstancedMesh(new THREE.IcosahedronGeometry(1, 1), flat(0xffffff, { roughness: 1 }), treeCount);
  const greens = [0x2f5d2c, 0x3a6f33, 0x274f2a, 0x4a7d3a, 0x355f3a];
  for (let i = 0; i < treeCount; i++) {
    let x, y;
    do { x = (random() - 0.5) * 2 * (outerX + 75); y = (random() - 0.5) * 2 * (outerY + 65); } while (Math.abs(x) < outerX + 16.5 && Math.abs(y) < outerY + 6);
    const height = 3 + random() * 4, crown = 2.2 + random() * 2.6;
    dummy.rotation.set(Math.PI / 2, 0, 0); dummy.scale.set(1, height, 1);
    dummy.position.set(x, y, height / 2); dummy.updateMatrix();
    trunks.setMatrixAt(i, dummy.matrix);
    dummy.rotation.set(0, 0, random() * 6); dummy.scale.set(crown, crown, crown * (0.75 + random() * 0.3));
    dummy.position.set(x, y, height + crown * 0.55); dummy.updateMatrix();
    canopies.setMatrixAt(i, dummy.matrix);
    canopies.setColorAt(i, tint.setHex(greens[Math.floor(random() * greens.length)]));
  }
  group.add(trunks, canopies);

  // campus and apartment blocks beyond the trees
  const walls = [['#c9b08a', '#3c4a57'], ['#a8573f', '#2f3a44'], ['#e6e1d6', '#465564'], ['#b98c62', '#33404b'], ['#8f9aa3', '#2b3640']];
  const roofs = [0x8a4a3a, 0x6f7479, 0x9a5a42, 0x55595e];
  for (let i = 0; i < 34; i++) {
    const angle = (i / 34) * 2 * Math.PI + (random() - 0.5) * 0.12;
    const distance = 1 + random() * 0.55;
    const x = Math.cos(angle) * (outerX + 48) * distance, y = Math.sin(angle) * (outerY + 44) * distance;
    const w = 12 + random() * 16, d = 9 + random() * 9, h = 8 + random() * 16;
    const [wall, glass] = walls[Math.floor(random() * walls.length)];
    const map = windowTexture(wall, glass);
    map.repeat.set(Math.round(w / 3), Math.round(h / 3));
    const sides = new THREE.MeshStandardMaterial({ map, roughness: 0.85, metalness: 0 });
    const top = flat(roofs[Math.floor(random() * roofs.length)]);
    const block = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), [sides, sides, sides, sides, top, top]);
    // box side faces map their texture with "up" along y on the x faces; turn those to stand upright
    block.geometry.attributes.uv.needsUpdate = true;
    block.position.set(x, y, h / 2);
    block.rotation.z = Math.round(random() * 3) * 0.12;
    group.add(block);
  }

  return { group, reach: Math.hypot(outerX + 14, outerY + 6) };
}

// ---------------------------------------------------------------- venue: stadium
// A big enclosed football stadium in the style of the modern World Cup venues: two tiers of packed stands all the
// way round, LED ribbon boards, a roof with an opening over the pitch and a halo video board hanging in it.

function ribbonTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 2048; canvas.height = 64;
  const ctx = canvas.getContext('2d');
  const gradient = ctx.createLinearGradient(0, 0, canvas.width, 0);
  for (let i = 0; i <= 8; i++) gradient.addColorStop(i / 8, i % 2 ? '#1e40af' : '#0b1f6b');
  ctx.fillStyle = gradient; ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#f8fafc';
  ctx.font = '700 34px system-ui, -apple-system, "Segoe UI", sans-serif';
  ctx.textBaseline = 'middle';
  for (let x = 40; x < canvas.width; x += canvas.width / 3) {
    ctx.fillText('ROBOCUP', x, 34);
    ctx.fillStyle = '#facc15'; ctx.fillText('HUMANOID SOCCER', x + 190, 34); ctx.fillStyle = '#f8fafc';
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = THREE.RepeatWrapping;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return texture;
}

// A vertical band following a rounded rectangle, textured on the side facing the pitch (boards, ribbons, halo).
function bandGeometry(hx, hy, radius, z0, height, repeats) {
  const points = roundedRect(new THREE.Shape(), hx, hy, radius).getSpacedPoints(200);
  const positions = [], uvs = [], indices = [];
  points.forEach((point, i) => {
    // u runs against the outline so that text reads left to right from inside the ring
    const u = -(i / (points.length - 1)) * repeats;
    positions.push(point.x, point.y, z0, point.x, point.y, z0 + height);
    uvs.push(u, 0, u, 1);
    if (i > 0) indices.push(2 * i - 2, 2 * i, 2 * i - 1, 2 * i - 1, 2 * i, 2 * i + 1);
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function buildStadium(halfX, halfY, game) {
  const group = new THREE.Group();
  const random = seededRandom(7);
  const corner = 3;
  const ring = (offset) => [halfX + offset, halfY + offset, corner + offset];
  const ringShape = (outer, inner) => {
    const shape = roundedRect(new THREE.Shape(), ...ring(outer));
    shape.holes.push(roundedRect(new THREE.Path(), ...ring(inner)));
    return shape;
  };
  const led = (map, intensity) => new THREE.MeshStandardMaterial({ map, emissive: 0xffffff, emissiveMap: map, emissiveIntensity: intensity, roughness: 0.5, metalness: 0, side: THREE.DoubleSide });

  // apron between the pitch and the stands
  const apron = new THREE.Mesh(new THREE.ShapeGeometry(roundedRect(new THREE.Shape(), ...ring(3)), 12), new THREE.MeshStandardMaterial({ color: 0x1c2a52, roughness: 0.85 }));
  apron.position.z = -0.01;
  apron.receiveShadow = true;
  group.add(apron);

  // pitch-side LED boards
  const boards = boardMap.clone(); boards.needsUpdate = true;
  group.add(new THREE.Mesh(bandGeometry(...ring(0.9), 0, 0.85, Math.round((halfX + halfY) / 2.5)), led(boards, 0.55)));

  // the stands: concrete steps with one spectator per seat
  const concrete = new THREE.MeshStandardMaterial({ color: 0x3a4150, roughness: 0.95, metalness: 0 });
  const rows = [];
  const tier = (count, firstOffset, firstHeight, rise) => {
    for (let i = 0; i < count; i++) rows.push({ offset: firstOffset + i * 0.8, height: firstHeight + i * rise });
  };
  tier(12, 2.6, 1.1, 0.42);   // lower tier
  tier(10, 12.8, 8.6, 0.52);  // upper tier, behind the ribbon board
  for (const row of rows) {
    const step = new THREE.Mesh(new THREE.ExtrudeGeometry(ringShape(row.offset + 0.8, row.offset), { depth: row.height, bevelEnabled: false, curveSegments: 6 }), concrete);
    step.receiveShadow = true;
    group.add(step);
  }
  // wall and ribbon board between the tiers, back wall behind the upper tier
  const wall = new THREE.Mesh(new THREE.ExtrudeGeometry(ringShape(12.8, 12.2), { depth: 8.6, bevelEnabled: false, curveSegments: 6 }), new THREE.MeshStandardMaterial({ color: 0x14181f, roughness: 0.9 }));
  const back = new THREE.Mesh(new THREE.ExtrudeGeometry(ringShape(21.6, 20.8), { depth: 17.5, bevelEnabled: false, curveSegments: 6 }), new THREE.MeshStandardMaterial({ color: 0x1b2029, roughness: 0.9 }));
  group.add(wall, back);
  const ribbon = ribbonTexture();
  group.add(new THREE.Mesh(bandGeometry(...ring(12.15), 6.9, 1.2, 6), led(ribbon, 0.9)));

  // the crowd: mostly in the two teams' colours, each team's supporters behind "their" half
  let seats = 0;
  const seatPoints = rows.map((row) => roundedRect(new THREE.Shape(), ...ring(row.offset + 0.4)).getSpacedPoints(Math.round((4 * (halfX + halfY) + 8 * row.offset) / 0.55)));
  for (const points of seatPoints) seats += points.length;
  const crowd = new THREE.InstancedMesh(new THREE.CapsuleGeometry(0.14, 0.26, 2, 6), new THREE.MeshStandardMaterial({ roughness: 0.9, metalness: 0 }), seats);
  const crowdHeads = new THREE.InstancedMesh(new THREE.SphereGeometry(0.095, 7, 5), new THREE.MeshStandardMaterial({ roughness: 0.8, metalness: 0 }), seats);
  const skins = [0xf1c9a5, 0xd9a47a, 0xa8714a, 0x6f4a32];
  const dummy = new THREE.Object3D(), tint = new THREE.Color(), other = new THREE.Color();
  const neutrals = [0xf8fafc, 0x111827, 0xfacc15, 0x64748b, 0xe2e8f0];
  let seat = 0;
  seatPoints.forEach((points, r) => {
    for (const point of points) {
      if (random() < 0.07) continue; // empty seat
      const sx = point.x + (random() - 0.5) * 0.1, sy = point.y + (random() - 0.5) * 0.1;
      dummy.position.set(sx, sy, rows[r].height + 0.27);
      dummy.rotation.set(Math.PI / 2, 0, 0);
      dummy.updateMatrix();
      crowd.setMatrixAt(seat, dummy.matrix);
      dummy.position.z = rows[r].height + 0.62;
      dummy.updateMatrix();
      crowdHeads.setMatrixAt(seat, dummy.matrix);
      crowdHeads.setColorAt(seat, tint.setHex(skins[Math.floor(random() * skins.length)]));
      const fan = random() < 0.62 ? teamColour(game, point.x < 0 ? 0 : 1) : neutrals[Math.floor(random() * neutrals.length)];
      tint.setHex(fan).lerp(other.setHex(0x000000), random() * 0.35);
      crowd.setColorAt(seat, tint);
      seat++;
    }
  });
  crowd.count = crowdHeads.count = seat;
  group.add(crowd, crowdHeads);

  // roof with an opening over the pitch, a ring of floodlights along its edge
  const roofZ = 18.5, opening = 4.2;
  const roof = new THREE.Mesh(new THREE.ShapeGeometry(ringShape(23, opening), 16), new THREE.MeshStandardMaterial({ color: 0x0d1016, roughness: 0.8, metalness: 0.3, side: THREE.DoubleSide }));
  roof.position.z = roofZ;
  group.add(roof);
  const rim = roundedRect(new THREE.Shape(), ...ring(opening + 0.6)).getSpacedPoints(72);
  const lamps = new THREE.InstancedMesh(new THREE.SphereGeometry(0.22, 8, 6), new THREE.MeshBasicMaterial({ color: 0xfff6dd }), rim.length);
  rim.forEach((point, i) => { dummy.position.set(point.x, point.y, roofZ - 0.25); dummy.rotation.set(0, 0, 0); dummy.updateMatrix(); lamps.setMatrixAt(i, dummy.matrix); });
  group.add(lamps);
  // roof trusses, radiating from the opening
  const steel = new THREE.MeshStandardMaterial({ color: 0x2b313c, roughness: 0.5, metalness: 0.7 });
  const outerRim = roundedRect(new THREE.Shape(), ...ring(21)).getSpacedPoints(36);
  const innerRim = roundedRect(new THREE.Shape(), ...ring(opening)).getSpacedPoints(36);
  outerRim.forEach((outer, i) => {
    const inner = innerRim[i];
    const length = Math.hypot(outer.x - inner.x, outer.y - inner.y);
    const truss = new THREE.Mesh(new THREE.BoxGeometry(length, 0.18, 0.5), steel);
    truss.position.set((outer.x + inner.x) / 2, (outer.y + inner.y) / 2, roofZ - 0.35);
    truss.rotation.z = Math.atan2(outer.y - inner.y, outer.x - inner.x);
    group.add(truss);
  });

  // halo video board hanging in the opening: the scoreboard, repeated all the way round
  const haloHeight = 3.2;
  const perimeter = 4 * (halfX + halfY) + 8 * opening;
  const halo = new THREE.Mesh(bandGeometry(...ring(opening - 0.2), roofZ - haloHeight - 0.4, haloHeight, Math.max(2, Math.round(perimeter / (haloHeight * scoreCanvas.width / scoreCanvas.height)))), led(scoreMap, 1.0));
  group.add(halo);

  return { group, reach: Math.hypot(halfX + 23, halfY + 23) };
}

// ---------------------------------------------------------------- pre-game ceremony
// Before kick-off (game state "initial") a giant flag of each team is held out over its half, with an emblem on
// the centre circle, as at the big tournaments.

function flagTexture(name, colour) {
  const canvas = document.createElement('canvas');
  canvas.width = 1024; canvas.height = 640;
  const ctx = canvas.getContext('2d');
  const base = new THREE.Color(colour);
  const dark = `#${base.clone().multiplyScalar(0.55).getHexString()}`;
  const light = base.getHSL({}).l > 0.75 ? '#1f2937' : '#f8fafc';
  ctx.fillStyle = cssColour(colour); ctx.fillRect(0, 0, 1024, 640);
  ctx.fillStyle = light; ctx.fillRect(0, 230, 1024, 180);
  ctx.fillStyle = dark; ctx.fillRect(0, 218, 1024, 12); ctx.fillRect(0, 410, 1024, 12);
  // a ring of stars in the hoist corner, and the team name across the middle band
  ctx.fillStyle = light;
  for (let i = 0; i < 8; i++) {
    const a = i / 8 * 2 * Math.PI, cx = 130 + Math.cos(a) * 62, cy = 110 + Math.sin(a) * 62;
    ctx.beginPath();
    for (let k = 0; k < 10; k++) { const r = k % 2 ? 7 : 17, b = k / 10 * 2 * Math.PI - Math.PI / 2; ctx.lineTo(cx + Math.cos(b) * r, cy + Math.sin(b) * r); }
    ctx.fill();
  }
  ctx.fillStyle = cssColour(colour) === light ? dark : cssColour(colour);
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  fitText(ctx, String(name).toUpperCase(), 920, 130, 800);
  ctx.fillText(String(name).toUpperCase(), 512, 322);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return texture;
}

function emblemTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 512;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, 512, 512);
  const rings = [['#0b1f6b', 250], ['#f8fafc', 232], ['#1e40af', 220], ['#facc15', 150], ['#0b1f6b', 138]];
  for (const [colour, radius] of rings) { ctx.fillStyle = colour; ctx.beginPath(); ctx.arc(256, 256, radius, 0, 2 * Math.PI); ctx.fill(); }
  ctx.fillStyle = '#f8fafc';
  ctx.beginPath(); ctx.arc(256, 256, 84, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = '#111827';
  for (let i = 0; i < 5; i++) { const a = i / 5 * 2 * Math.PI - Math.PI / 2; ctx.beginPath(); ctx.arc(256 + Math.cos(a) * 52, 256 + Math.sin(a) * 52, 17, 0, 2 * Math.PI); ctx.fill(); }
  ctx.beginPath(); ctx.arc(256, 256, 20, 0, 2 * Math.PI); ctx.fill();
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

let ceremony = null; // {group, flags, key}

function isPreGame(game) {
  if (!game) return false;
  return game.state ? game.state === 'INITIAL' : /\binitial\b|before[ _]kick/i.test(String(game.mode));
}

function buildCeremony(game) {
  const group = new THREE.Group();
  const flags = [];
  const width = fieldHalf.x * 0.6, height = fieldHalf.y * 1.0;
  const dummy = new THREE.Object3D();
  for (const side of [0, 1]) {
    const colour = teamColour(game, side);
    const centreX = (side === 0 ? -1 : 1) * fieldHalf.x * 0.44;
    const flag = new THREE.Mesh(new THREE.PlaneGeometry(width, height, 30, 18), new THREE.MeshStandardMaterial({ map: flagTexture(side === 0 ? game.left : game.right, colour), roughness: 0.75, metalness: 0, side: THREE.DoubleSide }));
    flag.position.set(centreX, 0, 0.45);
    flag.castShadow = flag.receiveShadow = true;
    flag.userData.rest = flag.geometry.attributes.position.array.slice();
    group.add(flag);
    flags.push(flag);

    // the people holding the flag out, all the way round its edge
    const holders = [];
    for (let x = -width / 2; x <= width / 2 + 0.01; x += width / 12) holders.push([x, -height / 2 - 0.2], [x, height / 2 + 0.2]);
    for (let y = -height / 2 + height / 6; y < height / 2 - 0.01; y += height / 6) holders.push([-width / 2 - 0.2, y], [width / 2 + 0.2, y]);
    const people = new THREE.InstancedMesh(new THREE.CapsuleGeometry(0.11, 0.42, 3, 8), new THREE.MeshStandardMaterial({ color: colour, roughness: 0.8 }), holders.length);
    const heads = new THREE.InstancedMesh(new THREE.SphereGeometry(0.085, 10, 8), new THREE.MeshStandardMaterial({ color: 0xd9a47a, roughness: 0.7 }), holders.length);
    holders.forEach(([x, y], i) => {
      dummy.rotation.set(Math.PI / 2, 0, 0);
      dummy.position.set(centreX + x, y, 0.32); dummy.updateMatrix(); people.setMatrixAt(i, dummy.matrix);
      dummy.position.set(centreX + x, y, 0.74); dummy.updateMatrix(); heads.setMatrixAt(i, dummy.matrix);
    });
    people.castShadow = heads.castShadow = true;
    group.add(people, heads);
  }
  const radius = Math.min(1.5, fieldHalf.y * 0.24);
  const emblem = new THREE.Mesh(new THREE.CircleGeometry(radius, 48), new THREE.MeshStandardMaterial({ map: emblemTexture(), transparent: true, roughness: 0.7, polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4 }));
  emblem.position.z = 0.012;
  emblem.receiveShadow = true;
  group.add(emblem);
  return { group, flags };
}

// Shows, hides and (re)builds the ceremony to match the game state, and lets the flags ripple.
function updateCeremony(game, seconds) {
  const wanted = isPreGame(game) && world;
  const key = wanted ? `${shownSceneId}|${game.left}|${game.right}|${teamColour(game, 0)}|${teamColour(game, 1)}` : '';
  if (ceremony && ceremony.key !== key) {
    ceremony.group.removeFromParent();
    ceremony = null;
  }
  if (wanted && !ceremony) {
    ceremony = { ...buildCeremony(game), key };
    world.add(ceremony.group);
  }
  if (!ceremony) return;
  for (const [index, flag] of ceremony.flags.entries()) {
    const position = flag.geometry.attributes.position, rest = flag.userData.rest;
    for (let i = 0; i < position.count; i++) {
      const x = rest[3 * i], y = rest[3 * i + 1];
      position.array[3 * i + 2] = 0.07 * Math.sin(1.7 * x + 2.1 * seconds + index) * Math.cos(1.3 * y + 1.4 * seconds) + 0.03 * Math.sin(4.1 * x - 3 * seconds);
    }
    position.needsUpdate = true;
    flag.geometry.computeVertexNormals();
  }
}

const ballMap = ballTexture();

// ---------------------------------------------------------------- scene construction

const meshData = new Map();       // mesh key -> {vert: Float32Array, face: Uint32Array}
const meshGeometries = new Map(); // mesh key -> BufferGeometry
const primitiveGeometries = new Map();
const materials = new Map();
const sceneDescriptions = new Map(); // scene id -> scene description

let world = null;          // root object of the currently shown scene
let shownSceneId = -1;
let bodyObjects = [];      // body id -> Object3D
let robotLabels = [];      // {sprite, body}
let ballObject = null;     // the object whose position the ball cameras follow
let fieldHalf = { x: 11, y: 7 };
let decorReach = 20;               // radius the shadows must cover for the venue dressing
let stadiumHalf = { x: 11, y: 7 }; // extent of the ground the surroundings are built around
let showCollision = false;
let venue = 'village';        // 'village' (UNSW Village Green) or 'stadium'
let shownGameForDecor = null; // the game state the venue was dressed for (team colours in the stands)
try { venue = localStorage.getItem('nusim-venue') || venue; } catch (error) { /* storage unavailable */ }
let showLabels = true;

function decodeBase64(text) {
  const binary = atob(text);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes.buffer;
}

function meshGeometry(key) {
  if (!meshGeometries.has(key)) {
    const data = meshData.get(key);
    if (!data) return null;
    const indexed = new THREE.BufferGeometry();
    indexed.setAttribute('position', new THREE.BufferAttribute(data.vert, 3));
    indexed.setIndex(new THREE.BufferAttribute(data.face, 1));
    const geometry = toCreasedNormals(indexed, Math.PI / 5);
    indexed.dispose();
    meshGeometries.set(key, geometry);
  }
  return meshGeometries.get(key);
}

function primitiveGeometry(geom) {
  const [a, b, c] = geom.size;
  const key = `${geom.type}:${a}:${b}:${c}`;
  if (primitiveGeometries.has(key)) return primitiveGeometries.get(key);

  let geometry;
  switch (geom.type) {
    case 'plane':
      geometry = new THREE.PlaneGeometry(2 * (a || 500), 2 * (b || 500));
      break;
    case 'sphere':
      geometry = new THREE.SphereGeometry(a, 48, 32);
      geometry.rotateX(Math.PI / 2); // texture poles along z
      break;
    case 'ellipsoid':
      geometry = new THREE.SphereGeometry(1, 40, 28);
      geometry.scale(a, b, c);
      break;
    case 'capsule':
      geometry = new THREE.CapsuleGeometry(a, 2 * b, 12, 28);
      geometry.rotateX(Math.PI / 2);
      break;
    case 'cylinder':
      geometry = new THREE.CylinderGeometry(a, a, Math.max(2 * b, 0.001), 40);
      geometry.rotateX(Math.PI / 2);
      break;
    case 'box':
      geometry = new THREE.BoxGeometry(Math.max(2 * a, 0.001), Math.max(2 * b, 0.001), Math.max(2 * c, 0.001));
      break;
    default:
      return null;
  }
  primitiveGeometries.set(key, geometry);
  return geometry;
}

function cached(key, create) {
  if (!materials.has(key)) materials.set(key, create());
  return materials.get(key);
}

// Field markings are thin, non-colliding boxes (and discs) lying on the ground.
function isMarking(geom) {
  return (geom.body === 0 && !geom.collide && (geom.type === 'box' || geom.type === 'cylinder') && geom.size[geom.type === 'box' ? 2 : 1] <= 0.002);
}

function isPitch(geom) {
  return geom.type === 'plane' && geom.name === 'floor';
}

function materialFor(geom, collision) {
  if (collision) {
    return cached('collision', () => new THREE.MeshBasicMaterial({ color: 0xff3df2, wireframe: true, transparent: true, opacity: 0.35 }));
  }
  if (isPitch(geom)) {
    // synthetic turf is one even green; the stadium's natural grass is mown in stripes
    return cached(`pitch:${venue}`, () => new THREE.MeshStandardMaterial({ map: venue === 'village' ? turfTexture : pitchTexture, roughness: 0.95, metalness: 0, envMapIntensity: 0.25 }));
  }
  if (isMarking(geom)) {
    return cached('line', () => new THREE.MeshStandardMaterial({ color: 0xf4f6f1, roughness: 0.9, metalness: 0, envMapIntensity: 0.2, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 }));
  }
  if (geom.name === 'ball') {
    return cached('ball', () => new THREE.MeshPhysicalMaterial({ map: ballMap, roughness: 0.35, metalness: 0, clearcoat: 0.5, clearcoatRoughness: 0.3 }));
  }
  const [r, g, b, alpha] = geom.rgba;
  if (alpha < 0.999 && geom.mat.includes('net')) {
    return cached('net', () => new THREE.MeshStandardMaterial({ map: netMap, transparent: true, alphaTest: 0.35, side: THREE.DoubleSide, roughness: 0.9, metalness: 0 }));
  }
  if (geom.mat.includes('post')) {
    return cached('post', () => new THREE.MeshPhysicalMaterial({ color: 0xf6f7f8, roughness: 0.3, metalness: 0.1, clearcoat: 0.6, clearcoatRoughness: 0.25 }));
  }
  return cached(`rgba:${r}:${g}:${b}:${alpha}`, () => {
    // moulded plastic shells: light parts get a glossy clear coat, dark parts a satin finish
    const luminance = 0.2126 * r + 0.7152 * g + 0.0722 * b;
    const material = new THREE.MeshPhysicalMaterial({
      color: new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace),
      roughness: luminance > 0.5 ? 0.32 : 0.42,
      metalness: luminance > 0.5 ? 0.05 : 0.35,
      clearcoat: luminance > 0.5 ? 0.7 : 0.25,
      clearcoatRoughness: 0.2,
    });
    if (alpha < 0.999) {
      material.transparent = true;
      material.opacity = alpha;
      material.depthWrite = false;
      material.side = THREE.DoubleSide;
    }
    return material;
  });
}

function labelSprite(text, color, penalised = false) {
  const canvas = document.createElement('canvas');
  canvas.width = penalised ? 256 : 128; canvas.height = 64;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = penalised ? 'rgba(185, 28, 28, 0.92)' : 'rgba(12, 16, 22, 0.78)';
  ctx.beginPath(); ctx.roundRect(8, 8, canvas.width - 16, 48, 16); ctx.fill();
  ctx.strokeStyle = `#${color.toString(16).padStart(6, '0')}`;
  ctx.lineWidth = 5; ctx.stroke();
  ctx.fillStyle = '#ffffff';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  fitText(ctx, text, canvas.width - 36, 32, 600);
  ctx.fillText(text, canvas.width / 2, 34);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true }));
  sprite.scale.set(penalised ? 0.72 : 0.36, 0.18, 1);
  sprite.renderOrder = 10;
  return sprite;
}

function buildScene(description) {
  if (world) scene.remove(world);
  world = new THREE.Group();
  bodyObjects = [];
  robotLabels = [];
  ballObject = null;

  // Bodies with dedicated visual geoms (robots) use their colliding geoms as collision shapes only.
  const hasVisual = new Set(description.geoms.filter((geom) => geom.collide === false).map((geom) => geom.body));
  const ballBodies = new Set(description.geoms.filter((geom) => geom.name === 'ball').map((geom) => geom.body));

  for (const body of description.bodies) {
    const object = new THREE.Group();
    object.name = body.name;
    bodyObjects.push(object);
    world.add(object);
  }

  for (const geom of description.geoms) {
    const isBall = geom.name === 'ball';
    if (ballBodies.has(geom.body) && !isBall) continue; // the ball is drawn from its sphere alone
    const isCollision = !isBall && (geom.group > 2 || (geom.collide === true && geom.body !== 0 && hasVisual.has(geom.body)));
    if (geom.rgba[3] <= 0.001 && !isCollision && !isBall) continue;

    const geometry = geom.type === 'mesh' ? meshGeometry(geom.mesh) : primitiveGeometry(geom);
    if (!geometry) continue;

    const body = description.bodies[geom.body];
    const mesh = new THREE.Mesh(geometry, materialFor(geom, isCollision));
    if (isBall) ballObject = mesh;
    mesh.position.fromArray(geom.pos);
    mesh.quaternion.set(geom.quat[1], geom.quat[2], geom.quat[3], geom.quat[0]);

    const isGround = geom.type === 'plane';
    const marking = isMarking(geom);
    mesh.castShadow = !isGround && !marking && !isCollision && (isBall || geom.rgba[3] > 0.5);
    mesh.receiveShadow = !isCollision;
    mesh.userData.collision = isCollision;
    mesh.userData.shell = geom.type === 'mesh' && !isCollision && 0.2126 * geom.rgba[0] + 0.7152 * geom.rgba[1] + 0.0722 * geom.rgba[2] > 0.5;
    mesh.visible = !isCollision || showCollision;
    if (marking) mesh.position.z += 0.003;

    if (isPitch(geom)) {
      fieldHalf = { x: geom.size[0] || 11, y: geom.size[1] || 7 };
      pitchTexture.repeat.set(fieldHalf.x, fieldHalf.y / 2);   // 1 m wide mowing stripes
      turfTexture.repeat.set(fieldHalf.x, fieldHalf.y / 2);
    }

    bodyObjects[geom.body].add(mesh);
  }

  // dress the surroundings to the full extent of the ground (pitch plus any border planes)
  let groundX = 0, groundY = 0;
  for (const geom of description.geoms) {
    if (geom.type !== 'plane' || geom.body !== 0 || !geom.size[0] || !geom.size[1]) continue;
    groundX = Math.max(groundX, Math.abs(geom.pos[0]) + geom.size[0]);
    groundY = Math.max(groundY, Math.abs(geom.pos[1]) + geom.size[1]);
  }
  if (groundX > 0 && groundY > 0 && groundX < 80 && groundY < 80) {
    stadiumHalf = { x: groundX, y: groundY };
    if (venue === 'village') {
      const village = buildVillageGreen(groundX, groundY);
      world.add(village.group);
      decorReach = village.reach;
    } else {
      const stadium = buildStadium(groundX, groundY, shownGameForDecor);
      world.add(stadium.group);
      decorReach = stadium.reach;
    }
    fitSun(decorReach);
  }
  {
    // an enclosed stadium is lit by its floodlights rather than the open sky
    const indoors = venue === 'stadium';
    skyLight.intensity = indoors ? 0.55 : 0.45;
    scene.environmentIntensity = indoors ? 0.4 : 0.7;
    sun.intensity = indoors ? 2.4 : 3.0;
    surround.visible = true;
  }

  scene.add(world);
  shownSceneId = description.id;
  rosterLabels.clear();
  ceremony = null; // it lived in the previous scene; updateCeremony() builds a new one if still needed
  shownGame = null; // have the game state applied to the new scene (jerseys, labels, ceremony)
}

// ---------------------------------------------------------------- stream handling

const frames = [];       // {t, scene, poses, game}
let latestGame = null;
let live = true;         // following the newest frame
let paused = false;
let playhead = 0;        // simulation time currently shown
let replayMode = false;  // showing a recording instead of a live stream

function handleText(text) {
  const message = JSON.parse(text);
  if (message.type === 'mesh') {
    meshData.set(message.key, { vert: new Float32Array(decodeBase64(message.vert)), face: new Uint32Array(decodeBase64(message.face)) });
  } else if (message.type === 'scene') {
    sceneDescriptions.set(message.id, message);
  } else if (message.type === 'game') {
    latestGame = message;
  } else if (message.type === 'referee') {
    showReferee(message);
  }
}

function handleBinary(buffer) {
  const view = new DataView(buffer);
  if (view.getUint32(0, true) !== 1) return;
  const bodyCount = view.getUint32(16, true);
  const poses = new Float32Array(buffer.slice(24, 24 + bodyCount * 28));
  const time = view.getFloat64(8, true);

  // the simulation time went backwards: a new simulation started, so the old timeline is void
  if (frames.length && time < frames[frames.length - 1].t) frames.length = 0;

  frames.push({ t: time, scene: view.getUint32(4, true), poses, game: latestGame });
  if (frames.length > MAX_FRAMES) frames.splice(0, 1000);
  $('hint').classList.add('hidden');
}

// Splits a byte stream in the recording format into messages, however the bytes are chunked.
class RecordParser {
  constructor() {
    this.pending = new Uint8Array(0);
    this.headerSeen = false;
    this.decoder = new TextDecoder();
  }

  // Returns false if the stream is not in the recording format.
  push(chunk) {
    const bytes = new Uint8Array(this.pending.length + chunk.length);
    bytes.set(this.pending);
    bytes.set(chunk, this.pending.length);

    let offset = 0;
    if (!this.headerSeen) {
      if (bytes.length < RECORDING_MAGIC.length) { this.pending = bytes; return true; }
      if (this.decoder.decode(bytes.subarray(0, RECORDING_MAGIC.length)) !== RECORDING_MAGIC) return false;
      this.headerSeen = true;
      offset = RECORDING_MAGIC.length;
    }

    const view = new DataView(bytes.buffer);
    while (offset + 5 <= bytes.length) {
      const kind = view.getUint8(offset);
      const length = view.getUint32(offset + 1, true);
      if (offset + 5 + length > bytes.length) break; // incomplete message: wait for more bytes
      const start = offset + 5;
      if (kind === 0) handleText(this.decoder.decode(bytes.subarray(start, start + length)));
      else handleBinary(bytes.buffer.slice(start, start + length));
      offset = start + length;
    }

    this.pending = bytes.slice(offset);
    return true;
  }
}

function setStatus(kind, text) {
  $('status').className = `panel ${kind}`;
  $('status').lastElementChild.textContent = text;
}

// Live data arrives as an endless HTTP download in the recording format.
let streamAbort = null;
async function connect() {
  if (replayMode) return;
  try {
    streamAbort = new AbortController();
    const response = await fetch('/stream', { signal: streamAbort.signal, cache: 'no-store' });
    if (!response.ok || !response.body) throw new Error('no stream');
    setStatus('live', 'LIVE');
    const parser = new RecordParser();
    const reader = response.body.getReader();
    for (;;) {
      const { done, value } = await reader.read();
      if (done || !parser.push(value)) break;
    }
  } catch (error) {
    // fall through to the retry below
  }
  if (replayMode) return;
  setStatus('', 'DISCONNECTED');
  setTimeout(connect, 2000);
}

function loadRecording(buffer, label) {
  const bytes = new Uint8Array(buffer);
  if (new TextDecoder().decode(bytes.subarray(0, RECORDING_MAGIC.length)) !== RECORDING_MAGIC) {
    alert('This file is not a NUSim recording.');
    return;
  }

  replayMode = true;
  $('referee').hidden = true;
  if (streamAbort) streamAbort.abort();
  frames.length = 0;
  latestGame = null;
  sceneDescriptions.clear();

  new RecordParser().push(bytes); // a truncated last message (e.g. the simulator was killed) is simply left out

  live = false;
  paused = false;
  playhead = frames.length ? frames[0].t : 0;
  setStatus('replay', `REPLAY${label ? ` · ${label}` : ''}`);
}

function saveRecording() {
  if (!frames.length) return;
  const encoder = new TextEncoder();
  const parts = [encoder.encode(RECORDING_MAGIC)];
  const header = (kind, length) => {
    const bytes = new Uint8Array(5);
    const view = new DataView(bytes.buffer);
    view.setUint8(0, kind);
    view.setUint32(1, length, true);
    return bytes;
  };
  const addText = (object) => {
    const payload = encoder.encode(JSON.stringify(object));
    parts.push(header(0, payload.length), payload);
  };
  const encodeBase64 = (array) => {
    const bytes = new Uint8Array(array.buffer, array.byteOffset, array.byteLength);
    let binary = '';
    for (let i = 0; i < bytes.length; i += 0x8000) binary += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000));
    return btoa(binary);
  };

  for (const [key, data] of meshData) addText({ type: 'mesh', key, vert: encodeBase64(data.vert), face: encodeBase64(data.face) });

  let sceneId = -1, game = null;
  for (const frame of frames) {
    if (frame.scene !== sceneId) { sceneId = frame.scene; addText(sceneDescriptions.get(sceneId)); }
    if (frame.game && frame.game !== game) { game = frame.game; addText(game); }
    const head = new DataView(new ArrayBuffer(24));
    head.setUint32(0, 1, true);
    head.setUint32(4, frame.scene, true);
    head.setFloat64(8, frame.t, true);
    head.setUint32(16, frame.poses.length / 7, true);
    parts.push(header(1, 24 + frame.poses.byteLength), new Uint8Array(head.buffer), new Uint8Array(frame.poses.buffer));
  }

  const link = document.createElement('a');
  link.href = URL.createObjectURL(new Blob(parts, { type: 'application/octet-stream' }));
  link.download = `match-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.rcsr`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 5000);
}

// ---------------------------------------------------------------- playback

function frameIndexAt(time) {
  let low = 0, high = frames.length - 1;
  while (low < high) {
    const mid = (low + high + 1) >> 1;
    if (frames[mid].t <= time) low = mid; else high = mid - 1;
  }
  return low;
}

const qa = new THREE.Quaternion(), qb = new THREE.Quaternion();
const ballPosition = new THREE.Vector3();

function showFrameAt(time) {
  const index = frameIndexAt(time);
  const frame = frames[index];

  if (frame.scene !== shownSceneId) {
    const description = sceneDescriptions.get(frame.scene);
    if (!description) return null;
    buildScene(description);
  }

  // blend towards the next frame for smooth motion between simulation updates
  const next = frames[index + 1];
  const blend = next && next.scene === frame.scene && next.t - frame.t < 0.25 && next.t > frame.t
    ? THREE.MathUtils.clamp((time - frame.t) / (next.t - frame.t), 0, 1) : 0;

  const a = frame.poses, b = blend > 0 ? next.poses : a;
  const count = Math.min(bodyObjects.length, a.length / 7);
  for (let i = 0; i < count; i++) {
    const o = i * 7;
    const object = bodyObjects[i];
    object.position.set(a[o] + (b[o] - a[o]) * blend, a[o + 1] + (b[o + 1] - a[o + 1]) * blend, a[o + 2] + (b[o + 2] - a[o + 2]) * blend);
    qa.set(a[o + 4], a[o + 5], a[o + 6], a[o + 3]);
    if (blend > 0) qa.slerp(qb.set(b[o + 4], b[o + 5], b[o + 6], b[o + 3]), blend);
    object.quaternion.copy(qa);
  }

  for (const label of robotLabels) {
    label.sprite.position.copy(bodyObjects[label.body].position);
    label.sprite.position.z += 0.62;
  }
  if (ballObject) {
    ballObject.parent.updateMatrixWorld();
    ballObject.getWorldPosition(ballPosition);
  }

  // keep everything that moves inside the shadow area (robots enter the game from outside the pitch)
  let needed = decorReach;
  for (const label of robotLabels) needed = Math.max(needed, bodyObjects[label.body].position.length() + 2);
  needed = Math.min(needed, 60);
  if (needed > shadowReach || needed < shadowReach - 4) fitSun(needed);

  return frame;
}

function formatTime(seconds, tenths = true) {
  const safe = Math.max(0, seconds);
  const minutes = Math.floor(safe / 60);
  const rest = safe - minutes * 60;
  return tenths ? `${minutes}:${rest.toFixed(1).padStart(4, '0')}` : `${String(minutes).padStart(2, '0')}:${String(Math.floor(rest)).padStart(2, '0')}`;
}

// " 25s" while a timed penalty runs, nothing for penalties that last until the referee ends them
const secondsLeft = (robot) => (robot.penaltySecs > 0 ? ` ${robot.penaltySecs}s` : '');

// The line under the scoreboard: the half, the game state (colour coded, with its countdown) and one chip per
// penalised robot in its team's colour.
function showStatusLine(game) {
  const line = $('mode');
  line.textContent = '';
  const chip = (className, ...children) => {
    const element = document.createElement('span');
    element.className = `chip ${className}`;
    element.append(...children);
    line.append(element);
    return element;
  };
  const bold = (text) => { const b = document.createElement('b'); b.textContent = text; return b; };

  const [phase, ...rest] = String(game.mode).split('|').map((part) => part.trim()).filter(Boolean);
  if (phase) chip('phase', phase);
  const state = game.state || rest[0] || '';
  if (state) {
    const element = chip(`state ${state.toLowerCase()}`, document.createElement('i'), state.toLowerCase());
    if (game.secondaryTime > 0) element.append(bold(`${game.secondaryTime}s`));
  }
  for (const robot of game.robots || []) {
    if (!robot.penalty) continue;
    const element = chip('penalty', document.createElement('i'), bold(`#${robot.player}`), robot.penalty.toLowerCase().replace(/_/g, ' '));
    if (robot.penaltySecs > 0) element.append(bold(`${robot.penaltySecs}s`));
    element.title = `${robot.side === 0 ? game.left : game.right} player ${robot.player}`;
    element.style.setProperty('--team', cssColour(teamColour(game, robot.side)));
  }
}

let shownGame = null;
function showGame(game) {
  if (!game || game === shownGame) return;
  shownGame = game;
  $('scoreboard').hidden = false;
  $('teamL').textContent = game.left;
  $('teamR').textContent = game.right;
  $('teamL').style.borderLeftColor = cssColour(teamColour(game, 0));
  $('teamR').style.borderRightColor = cssColour(teamColour(game, 1));
  $('score').textContent = `${game.scoreLeft} : ${game.scoreRight}`;
  $('clock').textContent = formatTime(game.time, false);
  showStatusLine(game);
  drawScoreboard(game);
  applyRoster(game);

  // the stands wear the teams' colours: dress the stadium again when the teams change
  const decorKey = `${teamColour(game, 0)}|${teamColour(game, 1)}`;
  if (venue === 'stadium' && decorKey !== shownDecorKey) {
    shownDecorKey = decorKey;
    shownGameForDecor = game;
    shownSceneId = -1;
  }
}
let shownDecorKey = `${TEAM_COLORS[0]}|${TEAM_COLORS[1]}`;

// Robots the GameController governs: jersey in the team colour, a label with the player number, and a red
// label with the reason and the remaining time while penalised.
const rosterLabels = new Map(); // body name -> {key, colour, label}
function applyRoster(game) {
  if (!game.robots || !world) return;
  for (const robot of game.robots) {
    const body = bodyObjects.findIndex((object) => object.name === robot.body);
    if (body < 0) continue;
    const colour = teamColour(game, robot.side);
    const key = `${robot.player}|${robot.penalty}|${robot.penaltySecs}|${colour}`;
    let entry = rosterLabels.get(robot.body);
    if (entry && entry.key === key) continue;

    if (!entry || entry.colour !== colour) {
      const jersey = cached(`jersey:${colour}`, () => new THREE.MeshPhysicalMaterial({ color: colour, roughness: 0.3, metalness: 0.1, clearcoat: 0.8, clearcoatRoughness: 0.18 }));
      for (const mesh of bodyObjects[body].children) if (mesh.userData.shell) mesh.material = jersey;
    }
    if (entry) {
      entry.label.sprite.removeFromParent();
      robotLabels.splice(robotLabels.indexOf(entry.label), 1);
    }
    const text = robot.penalty ? `${robot.player} · ${robot.penalty.toLowerCase().replace(/_/g, ' ')}${secondsLeft(robot)}` : String(robot.player);
    const label = { sprite: labelSprite(text, colour, Boolean(robot.penalty)), body };
    label.sprite.visible = showLabels;
    world.add(label.sprite);
    robotLabels.push(label);
    rosterLabels.set(robot.body, { key, colour, label });
  }
}

// The simulators report the game state as "a | b | c"; keep the words, drop flags and raw timers.
function modeText(mode) {
  return String(mode).split('|').map((part) => part.trim()).filter((part) => part && !/^(False|True|none|-?\d+(\.\d+)?)$/.test(part)).map((part) => part.replace(/_/g, ' ')).join('  ·  ');
}

// ---------------------------------------------------------------- referee panel
// With a simulator that runs the GameController itself (NUSim --referee), the panel operates it: each button
// posts a decision, and the simulator reports back which decisions the rules allow at the moment.

let refereeStatus = null;
let refereeToastTimer = 0;

function sendReferee(query) {
  fetch(`/referee?${query}`, { method: 'POST' }).catch(() => {});
}

function showReferee(status) {
  const answered = refereeStatus && status.seq !== refereeStatus.seq;
  refereeStatus = status;
  $('referee').hidden = replayMode;
  const legal = new Set(status.legal);
  for (const button of document.querySelectorAll('#referee [data-legal]')) button.disabled = !legal.has(button.dataset.legal);
  // only one of stop / resume applies at a time
  document.querySelector('#referee [data-legal=stop]').hidden = legal.has('resume');
  document.querySelector('#referee [data-legal=resume]').hidden = !legal.has('resume');
  if (answered) {
    $('refToast').textContent = status.ok ? '' : status.text.replace(/_/g, ' ');
    clearTimeout(refereeToastTimer);
    refereeToastTimer = setTimeout(() => { $('refToast').textContent = ''; }, 3500);
  }
}

// Team names, the game state and one button per player, from the game state.
let refereePlayersKey = '';
function showRefereeGame(game) {
  if (!refereeStatus) return;
  for (const span of document.querySelectorAll('#referee [data-team]')) span.textContent = span.dataset.team === '0' ? game.left : game.right;
  $('refState').textContent = `${game.state || ''}${game.secondaryTime > 0 ? ` · ${game.secondaryTime}s` : ''}`;
  $('referee').style.setProperty('--left', cssColour(teamColour(game, 0)));
  $('referee').style.setProperty('--right', cssColour(teamColour(game, 1)));

  const robots = game.robots || [];
  const key = robots.map((robot) => `${robot.side}${robot.player}${robot.penalty}${robot.penaltySecs}`).join('|') + game.left + game.right;
  if (key === refereePlayersKey) return;
  refereePlayersKey = key;
  const box = $('refPlayers');
  box.textContent = '';
  for (const side of [0, 1]) {
    const team = document.createElement('div');
    team.className = 'team';
    team.textContent = side === 0 ? game.left : game.right;
    const row = document.createElement('div');
    row.className = 'row players';
    for (const robot of robots.filter((r) => r.side === side)) {
      const button = document.createElement('button');
      const sideName = side === 0 ? 'home' : 'away';
      button.className = `${sideName}${robot.penalty ? ' out' : ''}`;
      button.textContent = robot.penalty ? `${robot.player} ✕${secondsLeft(robot)}` : String(robot.player);
      button.title = robot.penalty ? `${robot.penalty.toLowerCase().replace(/_/g, ' ')} - click to release` : 'Click to penalise with the selected call';
      button.addEventListener('click', () => sendReferee(robot.penalty
        ? `action=unpenalize&side=${sideName}&player=${robot.player}&arg=force`
        : `action=penalize&side=${sideName}&player=${robot.player}&arg=${$('refCall').value}`));
      row.append(button);
    }
    box.append(team, row);
  }
}

for (const button of document.querySelectorAll('#referee [data-ref]')) {
  button.addEventListener('click', () => sendReferee(button.dataset.ref + (button.dataset.arg ? `&arg=${$(button.dataset.arg).value}` : '')));
}
$('refToggle').addEventListener('click', () => {
  const collapsed = $('referee').classList.toggle('collapsed');
  $('refToggle').textContent = collapsed ? '+' : '–';
});

// ---------------------------------------------------------------- cameras

let cameraMode = 'free';
const previousBall = new THREE.Vector3();
const smoothTarget = new THREE.Vector3();
const scratch = new THREE.Vector3();

function setCameraMode(mode) {
  cameraMode = mode;
  controls.enabled = mode === 'free' || mode === 'follow';
  camera.fov = mode === 'tv' ? 24 : mode === 'top' ? 30 : 45;
  camera.updateProjectionMatrix();
  smoothTarget.copy(ballPosition);
  previousBall.copy(ballPosition);
  if (mode === 'free') camera.up.set(0, 0, 1);
  for (const button of document.querySelectorAll('#cams button')) button.classList.toggle('on', button.dataset.cam === mode);
}

function updateCamera(dt) {
  const ease = 1 - Math.exp(-dt * 4);

  if (cameraMode === 'follow') {
    // move camera and orbit centre along with the ball, so the user keeps control of the viewing angle
    scratch.copy(ballPosition).sub(previousBall);
    scratch.z = 0;
    camera.position.add(scratch);
    controls.target.add(scratch);
    controls.target.z = THREE.MathUtils.lerp(controls.target.z, 0.3, ease);
    previousBall.copy(ballPosition);
    controls.update();
  } else if (cameraMode === 'tv') {
    smoothTarget.lerp(scratch.set(ballPosition.x, ballPosition.y, 0.3), ease);
    camera.up.set(0, 0, 1);
    camera.position.set(smoothTarget.x * 0.35, -(stadiumHalf.y + (venue === 'stadium' ? 10.5 : 12)), venue === 'stadium' ? 8.2 : 9);
    camera.lookAt(smoothTarget);
  } else if (cameraMode === 'top') {
    const height = Math.max(fieldHalf.x / camera.aspect, fieldHalf.y) / Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * 1.04;
    camera.up.set(0, 1, 0);
    camera.position.set(0, 0, height);
    camera.lookAt(0, 0, 0);
  } else {
    controls.update();
  }
}

// ---------------------------------------------------------------- user interface

function setPaused(value) {
  paused = value;
  if (paused) live = false;
  $('play').textContent = paused ? '▶' : '⏸';
}

function goLive() {
  if (replayMode || !frames.length) return;
  live = true;
  setPaused(false);
  playhead = frames[frames.length - 1].t - LIVE_DELAY;
}

function stepFrames(direction) {
  if (!frames.length) return;
  setPaused(true);
  const index = THREE.MathUtils.clamp(frameIndexAt(playhead) + direction, 0, frames.length - 1);
  playhead = frames[index].t;
}

let seeking = false;
$('seek').addEventListener('input', () => {
  if (!frames.length) return;
  seeking = true;
  live = false;
  const first = frames[0].t, last = frames[frames.length - 1].t;
  playhead = first + Number($('seek').value) * (last - first);
});
$('seek').addEventListener('change', () => { seeking = false; });
$('venue').value = venue;
$('venue').addEventListener('change', () => {
  venue = $('venue').value;
  try { localStorage.setItem('nusim-venue', venue); } catch (error) { /* storage unavailable */ }
  shownSceneId = -1; // rebuild the scene with the other venue
});
$('play').addEventListener('click', () => setPaused(!paused));
$('golive').addEventListener('click', goLive);
$('save').addEventListener('click', saveRecording);
$('open').addEventListener('click', () => $('file').click());
$('file').addEventListener('change', async () => {
  const file = $('file').files[0];
  if (file) loadRecording(await file.arrayBuffer(), file.name);
});
for (const button of document.querySelectorAll('#cams button')) button.addEventListener('click', () => setCameraMode(button.dataset.cam));

window.addEventListener('keydown', (event) => {
  if (event.target instanceof HTMLSelectElement || event.metaKey || event.ctrlKey) return;
  switch (event.key) {
    case ' ': event.preventDefault(); setPaused(!paused); break;
    case 'ArrowLeft': stepFrames(event.shiftKey ? -25 : -1); break;
    case 'ArrowRight': stepFrames(event.shiftKey ? 25 : 1); break;
    case 'l': case 'L': goLive(); break;
    case '1': setCameraMode('free'); break;
    case '2': setCameraMode('follow'); break;
    case '3': setCameraMode('tv'); break;
    case '4': setCameraMode('top'); break;
    case 'c': case 'C':
      showCollision = !showCollision;
      world?.traverse((object) => { if (object.userData.collision) object.visible = showCollision; });
      break;
    case 'n': case 'N':
      showLabels = !showLabels;
      for (const label of robotLabels) label.sprite.visible = showLabels;
      break;
    default:
  }
});

window.addEventListener('dragover', (event) => { event.preventDefault(); $('drop').classList.add('on'); });
window.addEventListener('dragleave', () => $('drop').classList.remove('on'));
window.addEventListener('drop', async (event) => {
  event.preventDefault();
  $('drop').classList.remove('on');
  const file = event.dataTransfer?.files[0];
  if (file) loadRecording(await file.arrayBuffer(), file.name);
});

function resize() {
  const width = window.innerWidth, height = window.innerHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

// ---------------------------------------------------------------- main loop

const clock = new THREE.Clock();

function tick() {
  const dt = Math.min(clock.getDelta(), 0.1);

  if (frames.length) {
    const first = frames[0].t, last = frames[frames.length - 1].t;

    if (live) {
      // follow the stream slightly delayed and gently correct drift instead of jumping
      const target = last - LIVE_DELAY;
      playhead += dt;
      const error = target - playhead;
      playhead = Math.abs(error) > 0.5 ? target : playhead + error * Math.min(1, dt * 3);
    } else if (!paused && !seeking) {
      playhead += dt * Number($('speed').value);
      if (playhead >= last) {
        playhead = last;
        if (replayMode) setPaused(true);
      }
    }
    playhead = THREE.MathUtils.clamp(playhead, first, last);

    const frame = showFrameAt(playhead);
    if (frame) showGame(frame.game);
    updateCeremony(frame ? frame.game : null, performance.now() / 1000);
    if (latestGame) showRefereeGame(latestGame); // the panel always acts on the live game, wherever the playhead is

    if (!seeking) $('seek').value = last > first ? (playhead - first) / (last - first) : 1;
    $('tnow').textContent = formatTime(playhead);
    $('tend').textContent = formatTime(last);
    $('golive').classList.toggle('on', live);
    $('golive').style.display = replayMode ? 'none' : '';
  }

  updateCamera(dt);
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

setCameraMode('free');
drawScoreboard(null);

// handle for the browser console and automated checks
window.nusimViewer = { camera, controls, scene, frames, bodies: () => bodyObjects };

const replayUrl = new URLSearchParams(location.search).get('replay');
if (replayUrl) {
  replayMode = true;
  setStatus('replay', 'LOADING');
  fetch(replayUrl)
    .then((response) => (response.ok ? response.arrayBuffer() : Promise.reject(new Error(response.statusText))))
    .then((buffer) => loadRecording(buffer, ''))
    .catch((error) => setStatus('', `FAILED TO LOAD: ${error.message}`));
} else {
  connect();
}

tick();
