/* XVI Robotics — lunar base scene: a realistic WebGL rendering of a humanoid
   robot walking on the moon past a SpaceX-style base. Self-hosted Three.js,
   no external requests. */
import * as THREE from './vendor/three.module.min.js';
import { createH2Robot, GAIT } from './h2-robot.js';

var TAU = Math.PI * 2;

/* ---------- deterministic hash (stable crater field) ---------- */
function hash2(i, j) {
  var n = Math.sin(i * 127.1 + j * 311.7) * 43758.5453;
  return n - Math.floor(n);
}

// treadmill speed locked to the H2 walk stride (m/s)
var SPEED = GAIT.stepLen / (GAIT.STANCE * GAIT.CYC);

/* ---------- procedural textures ---------- */
function glowTexture(inner, outer) {
  var c = document.createElement('canvas');
  c.width = c.height = 128;
  var g = c.getContext('2d');
  var grad = g.createRadialGradient(64, 64, 4, 64, 64, 62);
  grad.addColorStop(0, inner);
  grad.addColorStop(0.4, outer);
  grad.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

function dustTexture() {
  var c = document.createElement('canvas');
  c.width = c.height = 64;
  var g = c.getContext('2d');
  var grad = g.createRadialGradient(32, 32, 2, 32, 32, 30);
  grad.addColorStop(0, 'rgba(190,196,205,0.55)');
  grad.addColorStop(0.6, 'rgba(160,168,178,0.22)');
  grad.addColorStop(1, 'rgba(150,158,168,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}

function earthTexture() {
  var c = document.createElement('canvas');
  c.width = 512; c.height = 256;
  var g = c.getContext('2d');
  var grad = g.createLinearGradient(0, 0, 0, 256);
  grad.addColorStop(0, '#9db8d8');
  grad.addColorStop(0.12, '#3f74c4');
  grad.addColorStop(0.5, '#2a5cb0');
  grad.addColorStop(0.88, '#3f74c4');
  grad.addColorStop(1, '#9db8d8');
  g.fillStyle = grad;
  g.fillRect(0, 0, 512, 256);
  // continents: blobby random walks, seeded, muted tones
  g.fillStyle = 'rgba(104,118,84,0.7)';
  for (var k = 0; k < 14; k++) {
    var x = hash2(k, 1) * 512, y = 46 + hash2(k, 2) * 164;
    for (var s = 0; s < 26; s++) {
      var r = 5 + hash2(k, s + 3) * 13;
      g.beginPath();
      g.arc((x + 512) % 512, y, r, 0, TAU);
      g.fill();
      x += (hash2(k, s + 40) - 0.5) * 26;
      y += (hash2(k, s + 80) - 0.5) * 18;
      y = Math.max(34, Math.min(222, y));
    }
  }
  // clouds: fine streaks
  g.fillStyle = 'rgba(255,255,255,0.34)';
  for (k = 0; k < 130; k++) {
    var cx2 = hash2(k, 200) * 512, cy2 = 12 + hash2(k, 300) * 232;
    var w = 8 + hash2(k, 400) * 30, h = 2 + hash2(k, 500) * 4;
    g.beginPath();
    g.ellipse(cx2, cy2, w, h, (hash2(k, 600) - 0.5) * 0.6, 0, TAU);
    g.fill();
  }
  // polar caps
  g.fillStyle = 'rgba(240,248,255,0.9)';
  g.fillRect(0, 0, 512, 14);
  g.fillRect(0, 242, 512, 14);
  var tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/* ---------- terrain: periodic crater field, flat corridor for the walker ---------- */
var TILE = 240, TILE_W = 200;
var CELL = 11, NXC = Math.round(TILE / CELL);
var craterCells = null;
function buildCraterCells() {
  // precomputed once: ~22x24 cells instead of 4 hashes per cell per vertex
  craterCells = [];
  var JM = Math.ceil(TILE_W / 2 / CELL) + 2;
  for (var iw = 0; iw < NXC; iw++) {
    var row = craterCells[iw] = {};
    for (var j = -JM; j <= JM; j++) {
      if (hash2(iw, j) < 0.45) { row[j] = null; continue; }
      row[j] = {
        fx: 0.2 + 0.6 * hash2(iw + 7, j + 3),
        fz: 0.2 + 0.6 * hash2(iw + 1, j + 9),
        rad: 1.4 + 3.4 * hash2(iw + 4, j + 6)
      };
    }
  }
}
function terrainH(x, z) {
  // periodic in x with period TILE so leapfrogging tiles join seamlessly
  if (!craterCells) buildCraterCells();
  var h = 0.16 * Math.sin((x * 2.1 + 1.7) / TILE * TAU) + 0.13 * Math.sin(z * 0.055 + 0.4) + 0.1 * Math.sin((x * 3.0 + z * 0.9) / TILE * TAU + 2.2);
  var ci = Math.floor(x / CELL), cj = Math.floor(z / CELL);
  for (var i = ci - 1; i <= ci + 1; i++) {
    var iw = ((i % NXC) + NXC) % NXC; // wrap cell hash in x
    var row = craterCells[iw];
    for (var j = cj - 1; j <= cj + 1; j++) {
      var cc = row[j];
      if (!cc) continue;
      var cx = (i + cc.fx) * CELL;
      var cz = (j + cc.fz) * CELL;
      var rad = cc.rad;
      var dx = x - cx, dz = z - cz;
      var d2 = dx * dx + dz * dz;
      var reach = rad * 1.5;
      if (d2 > reach * reach) continue;
      var d = Math.sqrt(d2);
      var depth = rad * 0.2;
      if (d < rad) {
        var q = 1 - (d / rad) * (d / rad);
        h -= depth * q * Math.sqrt(q);
      }
      var rim = (d - rad) / (rad * 0.22);
      h += depth * 0.55 * Math.exp(-rim * rim);
    }
  }
  var lat = Math.abs(z);
  if (lat < 6) {
    var k = lat < 2.2 ? 0 : (lat - 2.2) / 3.8;
    k = k * k * (3 - 2 * k);
    h *= 0.1 + 0.9 * k;
  }
  return h;
}

// terrain height under a world-group x position (tiles repeat every TILE)
function groundY(wx, z) {
  return terrainH(((wx + TILE / 2) % TILE + TILE) % TILE, z);
}

function buildTerrainGeometry(SEG_X, SEG_Z) {
  var geo = new THREE.PlaneGeometry(TILE, TILE_W, SEG_X, SEG_Z);
  geo.rotateX(-Math.PI / 2);
  var pos = geo.attributes.position;
  var colors = new Float32Array(pos.count * 3);
  for (var i = 0; i < pos.count; i++) {
    var x = pos.getX(i), z = pos.getZ(i);
    var h = terrainH(x + TILE / 2, z); // sample in [0, TILE)
    pos.setY(i, h);
    var sp = hash2(Math.round(x * 7.3), Math.round(z * 7.7));
    var b = 0.86 + h * 0.5 + (sp - 0.5) * 0.18;
    b = Math.max(0.55, Math.min(1.35, b));
    colors[i * 3] = b;
    colors[i * 3 + 1] = b;
    colors[i * 3 + 2] = b * 1.02;
  }
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geo.computeVertexNormals();
  return geo;
}

/* ---------- scene ---------- */
window.XVIMoonBase = function (canvas) {
  var reduce = false, COARSE = false;
  try { reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches; } catch (err) {}
  try { COARSE = window.matchMedia('(pointer: coarse)').matches; } catch (err) {}
  var DPR = window.devicePixelRatio || 1;
  var MAXPR = COARSE ? 1.5 : 2; // dpr-3 phones don't need a 2x buffer

  var renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: DPR <= 1.5, powerPreference: 'high-performance' });
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.98;
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  var scene = new THREE.Scene();
  scene.background = new THREE.Color(0x010205);
  scene.fog = new THREE.FogExp2(0x05070a, 0.009);

  var camera = new THREE.PerspectiveCamera(40, 1, 0.1, 900);

  /* lights */
  var sun = new THREE.DirectionalLight(0xfff3e2, 3.6);
  sun.position.set(14, 15, 13);
  sun.castShadow = true;
  sun.shadow.mapSize.set(COARSE ? 1024 : 2048, COARSE ? 1024 : 2048);
  sun.shadow.camera.left = -9; sun.shadow.camera.right = 9;
  sun.shadow.camera.top = 9; sun.shadow.camera.bottom = -9;
  sun.shadow.camera.near = 2; sun.shadow.camera.far = 80;
  sun.shadow.bias = -0.0004;
  scene.add(sun);
  scene.add(sun.target);

  var earthshine = new THREE.DirectionalLight(0x6f93c8, 0.35);
  earthshine.position.set(-30, 26, 38);
  scene.add(earthshine);
  scene.add(new THREE.AmbientLight(0x131a26, 0.6));

  /* sky: stars + earth (fixed to scene, not the scrolling world) */
  var starGeo = new THREE.BufferGeometry();
  var starN = 1700, starPos = new Float32Array(starN * 3), starCol = new Float32Array(starN * 3);
  for (var i = 0; i < starN; i++) {
    var u = Math.random() * 2 - 1, th = Math.random() * TAU;
    var sq = Math.sqrt(1 - u * u);
    var R = 420;
    starPos[i * 3] = sq * Math.cos(th) * R;
    starPos[i * 3 + 1] = Math.abs(u) * R * 0.9 + 6;
    starPos[i * 3 + 2] = sq * Math.sin(th) * R;
    var w = 0.55 + Math.random() * 0.45;
    var tint = Math.random();
    starCol[i * 3] = w * (tint > 0.92 ? 0.75 : 1);
    starCol[i * 3 + 1] = w * (tint > 0.92 ? 0.92 : 1);
    starCol[i * 3 + 2] = w * (tint > 0.85 ? 1.1 : 1);
  }
  starGeo.setAttribute('position', new THREE.BufferAttribute(starPos, 3));
  starGeo.setAttribute('color', new THREE.BufferAttribute(starCol, 3));
  var stars = new THREE.Points(starGeo, new THREE.PointsMaterial({ size: 1.6, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.95, depthWrite: false, fog: false }));
  scene.add(stars);

  var earth = new THREE.Mesh(new THREE.SphereGeometry(2.6, 48, 32), new THREE.MeshBasicMaterial({ map: earthTexture(), fog: false }));
  earth.position.set(-30, 10, -92); // earthrise behind the base
  scene.add(earth);
  var earthGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTexture('rgba(120,170,255,0.5)', 'rgba(70,120,220,0.16)'), blending: THREE.AdditiveBlending, depthWrite: false, fog: false }));
  earthGlow.position.copy(earth.position);
  earthGlow.scale.setScalar(8);
  scene.add(earthGlow);

  /* scrolling world */
  var world = new THREE.Group();
  scene.add(world);

  var terrainGeo = buildTerrainGeometry(COARSE ? 150 : 220, COARSE ? 110 : 160);
  var terrainMat = new THREE.MeshStandardMaterial({ color: 0x595b60, roughness: 1, metalness: 0, vertexColors: true });
  var tiles = [];
  for (i = 0; i < 3; i++) {
    var tile = new THREE.Mesh(terrainGeo, terrainMat);
    tile.receiveShadow = true;
    tile.position.x = i * TILE;
    world.add(tile);
    tiles.push(tile);
  }

  /* ---------- materials for hardware ---------- */
  var matDark = new THREE.MeshStandardMaterial({ color: 0x14181d, metalness: 0.7, roughness: 0.45 });
  var matHull = new THREE.MeshStandardMaterial({ color: 0xd2d5da, metalness: 0.45, roughness: 0.42 });
  var matHab = new THREE.MeshStandardMaterial({ color: 0xb9bcc2, metalness: 0.25, roughness: 0.6 });
  var matPanel = new THREE.MeshStandardMaterial({ color: 0x16243c, metalness: 0.5, roughness: 0.35 });
  var matVisor = new THREE.MeshStandardMaterial({ color: 0x05161c, metalness: 0.6, roughness: 0.2, emissive: 0x7ee0ff, emissiveIntensity: 2.6 });
  var matCyan = new THREE.MeshStandardMaterial({ color: 0x021014, emissive: 0x7ee0ff, emissiveIntensity: 2.4 });
  var matWindow = new THREE.MeshStandardMaterial({ color: 0x1a1410, emissive: 0xffd9a2, emissiveIntensity: 1.6 });
  var matRed = new THREE.MeshStandardMaterial({ color: 0x140404, emissive: 0xff4438, emissiveIntensity: 2.5 });

  var cyanGlowTex = glowTexture('rgba(126,224,255,0.65)', 'rgba(126,224,255,0.18)');
  var warmGlowTex = glowTexture('rgba(255,220,170,0.6)', 'rgba(255,190,120,0.15)');

  function glowSprite(tex, s) {
    var sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, blending: THREE.AdditiveBlending, depthWrite: false, transparent: true }));
    sp.scale.setScalar(s);
    return sp;
  }

  /* ---------- the robot: real Unitree H2 URDF meshes, loaded async ---------- */
  // outer holder owns world placement + the vertical walk bob; inner group spins
  // the URDF's Z-up model upright into the scene's Y-up frame
  var robot = new THREE.Group();
  scene.add(robot);
  var robotZ = new THREE.Group();
  robotZ.rotation.x = -Math.PI / 2;
  robot.add(robotZ);

  var rig = null, groundY0 = 0.78, chestGlow = null;
  createH2Robot().then(function (r) {
    rig = r;
    robotZ.add(rig.root);
    groundY0 = rig.LEG * 0.93 + 0.055; // hip height that drops the planted foot to y0
    // brand accents on the real head/torso links (URDF frame: +x fwd, +y left, +z up)
    chestGlow = glowSprite(cyanGlowTex, 0.14);
    chestGlow.position.set(0.12, 0, 0.30);
    if (rig.torsoGroup) rig.torsoGroup.add(chestGlow);
    kick();
  }).catch(function (e) { if (window.console) console.warn('H2 load failed', e); });

  /* ---------- the base (SpaceX-style render: domes + modules + ships) ---------- */
  var base = new THREE.Group();
  base.position.set(-14, 0, -56);
  world.add(base);

  // landing pad
  var PADX = -2, PADZ = -70;
  var pad = new THREE.Mesh(new THREE.CylinderGeometry(7, 7.4, 0.25, 40), matHab);
  pad.position.set(PADX, 0.02, PADZ);
  base.add(pad);
  var m4 = new THREE.Matrix4();
  var studs = new THREE.InstancedMesh(new THREE.SphereGeometry(0.16, 8, 6), matCyan, 10);
  var studPts = new Float32Array(10 * 3);
  for (i = 0; i < 10; i++) {
    var pa = i / 10 * TAU;
    var sx = PADX + Math.cos(pa) * 6.6, sz2 = PADZ + Math.sin(pa) * 6.6;
    m4.makeTranslation(sx, 0.2, sz2);
    studs.setMatrixAt(i, m4);
    studPts[i * 3] = sx; studPts[i * 3 + 1] = 0.4; studPts[i * 3 + 2] = sz2;
  }
  base.add(studs);
  var studGlowGeo = new THREE.BufferGeometry();
  studGlowGeo.setAttribute('position', new THREE.BufferAttribute(studPts, 3));
  base.add(new THREE.Points(studGlowGeo, new THREE.PointsMaterial({ map: cyanGlowTex, size: 1.6, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false })));

  // warm site lighting so structures read against the black sky
  var siteLight = new THREE.PointLight(0xffd9b0, 420, 70, 1.8);
  siteLight.position.set(2, 7, 6);
  base.add(siteLight);
  var siteLight2 = new THREE.PointLight(0x9db8d8, 200, 60, 1.8);
  siteLight2.position.set(-12, 5, 8);
  base.add(siteLight2);

  // main dome + small domes + tunnels
  var dome = new THREE.Mesh(new THREE.SphereGeometry(5.6, 36, 20, 0, TAU, 0, Math.PI / 2), matHab);
  base.add(dome);
  var domeRing = new THREE.Mesh(new THREE.TorusGeometry(4.6, 0.1, 8, 48), matWindow);
  domeRing.rotation.x = Math.PI / 2;
  domeRing.position.y = 1.7;
  base.add(domeRing);
  var d2 = new THREE.Mesh(new THREE.SphereGeometry(2.6, 28, 16, 0, TAU, 0, Math.PI / 2), matHab);
  d2.position.set(-3.2, 0, 7.6);
  base.add(d2);
  var d3 = d2.clone();
  d3.position.set(6.8, 0, 6.2);
  d3.scale.setScalar(0.8);
  base.add(d3);
  function tunnel(ax, az, bx, bz) {
    var dx = bx - ax, dz = bz - az;
    var len = Math.sqrt(dx * dx + dz * dz);
    var t = new THREE.Mesh(new THREE.CylinderGeometry(0.9, 0.9, len, 14), matHab);
    t.rotation.z = Math.PI / 2;
    t.rotation.y = -Math.atan2(dz, dx);
    t.position.set((ax + bx) / 2, 0.7, (az + bz) / 2);
    base.add(t);
  }
  tunnel(0, 0, -3.2, 7.6);
  tunnel(0, 0, 6.8, 6.2);

  // horizontal habitat cylinder (laid-down ship per the renders)
  var hab = new THREE.Mesh(new THREE.CylinderGeometry(1.9, 1.9, 13, 22), matHull);
  hab.rotation.z = Math.PI / 2;
  hab.position.set(-18, 1.35, 2.5);
  base.add(hab);
  var habCap = new THREE.Mesh(new THREE.SphereGeometry(1.9, 22, 14), matHull);
  habCap.position.set(-24.5, 1.35, 2.5);
  base.add(habCap);
  var habWin = new THREE.Mesh(new THREE.BoxGeometry(10, 0.16, 0.05), matWindow);
  habWin.position.set(-18, 1.85, 4.42);
  base.add(habWin);
  var saddles = new THREE.InstancedMesh(new THREE.BoxGeometry(1.6, 1.6, 3.8), matDark, 3);
  for (i = 0; i < 3; i++) {
    m4.makeTranslation(-23 + i * 5, 0.1, 2.5);
    saddles.setMatrixAt(i, m4);
  }
  base.add(saddles);

  // comm tower with blinking beacon
  var tower = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.16, 13, 10), matDark);
  tower.position.set(3.5, 6.5, -7);
  base.add(tower);
  var dish = new THREE.Mesh(new THREE.SphereGeometry(0.9, 16, 10, 0, TAU, 0, Math.PI / 2.6), matHab);
  dish.rotation.x = -Math.PI / 3;
  dish.position.set(3.5, 11.6, -7);
  base.add(dish);
  var beaconMesh = new THREE.Mesh(new THREE.SphereGeometry(0.12, 8, 6), matRed);
  beaconMesh.position.set(3.5, 13.1, -7);
  base.add(beaconMesh);
  var beaconGlow = glowSprite(glowTexture('rgba(255,90,70,0.7)', 'rgba(255,60,50,0.2)'), 1.8);
  beaconGlow.position.copy(beaconMesh.position);
  base.add(beaconGlow);

  // floodlight masts
  function floodMast(x, z, tx, tz) {
    var pole = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.1, 8, 8), matDark);
    pole.position.set(x, 4, z);
    base.add(pole);
    var lamp = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.22, 0.34), matDark);
    lamp.position.set(x, 8, z);
    base.add(lamp);
    var lg = glowSprite(warmGlowTex, 2.6);
    lg.position.set(x, 8, z);
    base.add(lg);
    var spot = new THREE.SpotLight(0xffe2b8, 900, 60, 0.62, 0.7, 1.7);
    spot.position.set(x, 8, z);
    spot.target.position.set(tx, 0, tz);
    base.add(spot);
    base.add(spot.target);
  }
  floodMast(PADX + 4, PADZ + 9, PADX, PADZ);
  floodMast(-6, 6.5, 0, 0);

  // solar field (instanced: 32 meshes -> 2 draw calls)
  var panelGeo = new THREE.BoxGeometry(2.2, 0.06, 1.3);
  var panels = new THREE.InstancedMesh(panelGeo, matPanel, 16);
  var plegs = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.05, 0.05, 0.8, 6), matDark, 16);
  var pq = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, -0.5));
  var pv = new THREE.Vector3(), ps = new THREE.Vector3(1, 1, 1);
  for (i = 0; i < 16; i++) {
    var prow = Math.floor(i / 8), pcol = i % 8;
    var px2 = -22 + pcol * 3.1 + prow * 1.2, pz2 = 12 + prow * 3.4;
    m4.compose(pv.set(px2, 0.8, pz2), pq, ps);
    panels.setMatrixAt(i, m4);
    m4.makeTranslation(px2, 0.4, pz2);
    plegs.setMatrixAt(i, m4);
  }
  base.add(panels, plegs);

  /* Starships on and beyond the pad */
  function starship(x, z, s) {
    var g = new THREE.Group();
    var body = new THREE.Mesh(new THREE.CylinderGeometry(2.4, 2.4, 24, 26), matHull);
    body.position.y = 13;
    g.add(body);
    var nose = new THREE.Mesh(new THREE.ConeGeometry(2.4, 7, 26), matHull);
    nose.position.y = 28.5;
    g.add(nose);
    for (var li = 0; li < 4; li++) {
      var la = li / 4 * TAU + 0.4;
      var leg = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.16, 3.4, 8), matDark);
      leg.position.set(Math.cos(la) * 2.2, 1.4, Math.sin(la) * 2.2);
      leg.rotation.z = Math.cos(la) * 0.5;
      leg.rotation.x = -Math.sin(la) * 0.5;
      g.add(leg);
    }
    var ring = new THREE.Mesh(new THREE.TorusGeometry(2.42, 0.07, 8, 40), matCyan);
    ring.rotation.x = Math.PI / 2;
    ring.position.y = 4.6;
    g.add(ring);
    var winB = new THREE.Mesh(new THREE.BoxGeometry(0.06, 5, 0.5), matWindow);
    winB.position.set(-2.42, 22, 0);
    g.add(winB);
    var winC = new THREE.Mesh(new THREE.BoxGeometry(0.5, 5, 0.06), matWindow);
    winC.position.set(0, 20, 2.42);
    g.add(winC);
    g.position.set(x, 0, z);
    g.scale.setScalar(s);
    base.add(g);
  }
  starship(PADX, PADZ, 0.7); // on the pad, full silhouette on the horizon
  starship(10, -78, 0.62);   // parked beyond

  /* foreground boulders */
  var rockGeo = new THREE.IcosahedronGeometry(1, 1);
  var rp = rockGeo.attributes.position;
  for (i = 0; i < rp.count; i++) {
    var rr = 1 + (hash2(i, 5) - 0.5) * 0.5;
    rp.setXYZ(i, rp.getX(i) * rr, rp.getY(i) * rr * 0.7, rp.getZ(i) * rr);
  }
  rockGeo.computeVertexNormals();
  var rocks = [];
  for (i = 0; i < 12; i++) {
    var rock = new THREE.Mesh(rockGeo, terrainMat);
    var rs = 0.12 + hash2(i, 11) * 0.55;
    rock.scale.setScalar(rs);
    var rz = (hash2(i, 13) - 0.5) * 40;
    if (Math.abs(rz) < 2.5) rz = 2.5 + hash2(i, 17) * 4;
    var rx2 = hash2(i, 19) * 110 - 20;
    // seat on the terrain; the +TILE wrap keeps the same ground height
    rock.position.set(rx2, groundY(rx2, rz) + rs * 0.25, rz);
    rock.receiveShadow = true;
    world.add(rock);
    rocks.push(rock);
  }

  /* footfall dust: small pool of sprites, parented to the world */
  var dustTex = dustTexture();
  var dustPool = [];
  for (i = 0; i < 14; i++) {
    var dsp = new THREE.Sprite(new THREE.SpriteMaterial({ map: dustTex, transparent: true, opacity: 0, depthWrite: false }));
    dsp.visible = false;
    world.add(dsp);
    dustPool.push({ sp: dsp, age: 99, life: 1, vy: 0, vx: 0 });
  }
  var dustIdx = 0;
  function puff(footX, footZ) {
    for (var pi = 0; pi < 3; pi++) {
      var d = dustPool[dustIdx];
      dustIdx = (dustIdx + 1) % dustPool.length;
      d.age = 0;
      d.life = 0.9 + Math.random() * 0.6;
      d.vy = 0.25 + Math.random() * 0.3;
      d.vx = -0.3 - Math.random() * 0.4;
      d.sp.position.set(footX + scroll + (Math.random() - 0.5) * 0.12, 0.05, footZ + (Math.random() - 0.5) * 0.12);
      d.sp.scale.setScalar(0.12 + Math.random() * 0.1);
      d.sp.visible = true;
    }
  }

  /* ---------- camera control ---------- */
  var yaw = 0.42, pitch = 0.085, yawV = 0, dist = 5.7, lookY = 1.02, lookX = -0.9;
  var dragging = false, lx = 0, ly = 0, fired = false, activeId = null, lastMoveT = 0;
  var W = 0, H = 0;

  function applyQuality() {
    renderer.setPixelRatio(quality === 2 ? Math.min(DPR, MAXPR) : quality === 1 ? Math.min(DPR, 1.25) : 1);
  }

  function resize() {
    W = window.innerWidth; H = window.innerHeight;
    renderer.setSize(W, H, false);
    applyQuality(); // respect the adaptive ladder, don't silently restore full res
    camera.aspect = W / H;
    // landscape phones use the desktop-style centered framing (matches the CSS)
    var desktopish = W > 880 || (H <= 560 && W > H);
    camera.fov = desktopish ? 40 : 46;
    dist = desktopish ? 5.7 : 7.2;
    lookY = desktopish ? 1.02 : 1.5; // phones: text on top, robot framed lower
    lookX = desktopish ? -0.9 : 0;
    camera.updateProjectionMatrix();
    kick();
  }

  /* ---------- animation ---------- */
  var prevLp = [0, 0.5];
  var scroll = 0, raf = 0, last = performance.now(), t0 = last;
  var ema = 16, quality = 2, maxQ = 2, drops = 0; // pixel-ratio ladder state

  var prevPh = 0.0;
  function pose(t) {
    if (!rig) return;
    // reduced motion: freeze at a double-support phase (both feet planted, mid-stride)
    var info = rig.update(reduce ? 0.05 * GAIT.CYC : t);
    robot.position.y = groundY0 + info.bob;
    if (!reduce) {
      var ph = info.phase;
      if (ph < prevPh) puff(0.05, -0.1);                 // left heel strike (phase wrap)
      if (prevPh < 0.5 && ph >= 0.5) puff(0.05, 0.1);    // right heel strike
      prevPh = ph;
    }
  }

  var beaconPhase = 0;

  function frame(now) {
    var gap = now - last;
    var dt = Math.min(gap / 1000, 0.05);
    ema += ((gap > 100 ? 100 : gap) - ema) * 0.05;
    last = now;
    var t = (now - t0) / 1000;

    // adaptive pixel ratio: demote under load, promote one tier at a time with a
    // wide hysteresis band, and stop returning to a tier that keeps failing
    if (quality > 0 && ema > (quality === 2 ? 40 : 50)) {
      if (quality === 2 && ++drops >= 2) maxQ = 1;
      quality--;
      applyQuality();
    } else if (quality < maxQ && ema < 15) {
      quality++;
      applyQuality();
    }

    if (!dragging && !reduce) {
      yaw += yawV * dt;
      yawV *= Math.pow(0.12, dt);
    }
    if (!reduce) scroll += SPEED * dt;

    world.position.x = -scroll;

    // wrap terrain tiles and props as they fall behind (all wrap distances are
    // multiples of TILE so everything re-seats on identical ground)
    for (var i = 0; i < 3; i++) {
      if (tiles[i].position.x - scroll < -TILE) tiles[i].position.x += TILE * 3;
    }
    if (base.position.x - scroll < -150) base.position.x += TILE * 2;
    for (i = 0; i < rocks.length; i++) {
      if (rocks[i].position.x - scroll < -30) rocks[i].position.x += TILE;
    }

    pose(t);

    // dust
    for (i = 0; i < dustPool.length; i++) {
      var d = dustPool[i];
      if (d.age > d.life) { d.sp.visible = false; continue; }
      d.age += dt;
      var u = d.age / d.life;
      d.sp.position.y += d.vy * dt;
      d.sp.position.x += d.vx * dt;
      d.sp.scale.setScalar(d.sp.scale.x + dt * 0.55);
      d.sp.material.opacity = 0.4 * (1 - u);
    }

    // beacon blink + chest light pulse
    beaconPhase = t % 1.6;
    var bOn = beaconPhase < 0.12 ? 1 : 0.06;
    matRed.emissiveIntensity = 2.5 * bOn;
    beaconGlow.material.opacity = bOn;
    var pulse = 0.75 + 0.25 * Math.sin(t * 2.2);
    if (chestGlow) chestGlow.material.opacity = pulse;
    matCyan.emissiveIntensity = 1.9 + 0.7 * pulse;

    earth.rotation.y = t * 0.008;

    // camera: orbit with gentle idle sway
    var yawR = yaw + (reduce ? 0 : 0.1 * Math.sin(t * 0.07));
    var cp = Math.cos(pitch), spv = Math.sin(pitch);
    camera.position.set(
      lookX + Math.sin(yawR) * dist * cp,
      lookY + 0.62 + spv * dist,
      Math.cos(yawR) * dist * cp
    );
    camera.lookAt(lookX, lookY, 0);

    renderer.render(scene, camera);

    if (!reduce || dragging) {
      raf = requestAnimationFrame(frame);
    } else {
      raf = 0;
    }
  }

  function kick() {
    if (!raf) {
      last = performance.now();
      raf = requestAnimationFrame(frame);
    }
  }

  function down(e) {
    if (e.button !== 0 || !e.isPrimary) return;
    dragging = true; activeId = e.pointerId; lx = e.clientX; ly = e.clientY;
    lastMoveT = e.timeStamp || performance.now();
    yawV = 0;
    canvas.style.cursor = 'grabbing';
    try { canvas.setPointerCapture(e.pointerId); } catch (err) {}
    if (!fired) { fired = true; window.dispatchEvent(new CustomEvent('xvi-orbit')); }
    kick();
  }
  function move(e) {
    if (!dragging || e.pointerId !== activeId) return;
    var dx = e.clientX - lx, dy = e.clientY - ly;
    lx = e.clientX; ly = e.clientY;
    var nowT = e.timeStamp || performance.now();
    var edt = (nowT - lastMoveT) / 1000;
    lastMoveT = nowT;
    yaw -= dx * 0.0055;
    if (edt > 0.0005) {
      var inst = (-dx * 0.0055) / (edt < 0.004 ? 0.004 : edt);
      if (inst > 5) inst = 5; else if (inst < -5) inst = -5;
      yawV = yawV * 0.6 + inst * 0.4;
    }
    pitch = Math.max(-0.12, Math.min(0.5, pitch + dy * 0.0035));
  }
  function up(e) {
    if (e && e.pointerId !== activeId) return;
    if (e && e.type === 'pointercancel') yawV = 0; // pinch start shouldn't fling the camera
    dragging = false;
    canvas.style.cursor = 'grab';
  }

  canvas.style.cursor = 'grab';
  canvas.style.touchAction = 'pinch-zoom';
  function onRestore() { kick(); }

  canvas.addEventListener('pointerdown', down);
  canvas.addEventListener('webglcontextrestored', onRestore);
  window.addEventListener('pointermove', move);
  window.addEventListener('pointerup', up);
  window.addEventListener('pointercancel', up);
  window.addEventListener('resize', resize);
  resize();
  kick();

  return function () {
    cancelAnimationFrame(raf);
    canvas.removeEventListener('pointerdown', down);
    canvas.removeEventListener('webglcontextrestored', onRestore);
    window.removeEventListener('pointermove', move);
    window.removeEventListener('pointerup', up);
    window.removeEventListener('pointercancel', up);
    window.removeEventListener('resize', resize);
    renderer.dispose();
  };
};

// the page's boot script may have given up polling on a slow connection —
// announce readiness so it can start the scene the moment the module lands
window.dispatchEvent(new Event('xvi-scene-ready'));
