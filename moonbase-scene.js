/* XVI Robotics — lunar base scene: a realistic WebGL rendering of a humanoid
   robot walking on the moon past a SpaceX-style base. Self-hosted Three.js,
   no external requests. */
import * as THREE from 'three';
import { createH2Robot, GAIT } from './h2-robot.js';
import { EffectComposer } from './vendor/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from './vendor/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from './vendor/jsm/postprocessing/UnrealBloomPass.js';
import { ShaderPass } from './vendor/jsm/postprocessing/ShaderPass.js';
import { OutputPass } from './vendor/jsm/postprocessing/OutputPass.js';

var TAU = Math.PI * 2;

/* ---------- deterministic hash (stable crater field) ---------- */
function hash2(i, j) {
  var n = Math.sin(i * 127.1 + j * 311.7) * 43758.5453;
  return n - Math.floor(n);
}

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

/* ---------- terrain: gentle crater field, fully periodic in BOTH axes so a
   3x3 grid of identical tiles can stream around a free-roaming walker ---------- */
var CELL = 11, NXC = 22, PER = NXC * CELL; // square period (~242m)
var craterCells = null;
function buildCraterCells() {
  craterCells = [];                       // NXC x NXC torus of craters
  for (var iw = 0; iw < NXC; iw++) {
    var row = craterCells[iw] = [];
    for (var jw = 0; jw < NXC; jw++) {
      row[jw] = hash2(iw * 3 + 1, jw * 5 + 2) < 0.5 ? null : {
        fx: 0.2 + 0.6 * hash2(iw + 7, jw + 3),
        fz: 0.2 + 0.6 * hash2(iw + 1, jw + 9),
        rad: 1.4 + 3.0 * hash2(iw + 4, jw + 6)
      };
    }
  }
}
function wrapc(i) { return ((i % NXC) + NXC) % NXC; }
function terrainH(x, z) {
  if (!craterCells) buildCraterCells();
  // integer harmonics of the period -> seamless across tile seams in x AND z
  var h = 0.34 * Math.sin(x / PER * TAU) + 0.28 * Math.sin(z / PER * TAU + 0.7)
        + 0.18 * Math.sin((x + z) / PER * TAU * 2 + 2.2)
        + 0.12 * Math.sin((x - z) / PER * TAU * 3 + 1.1);
  var ci = Math.floor(x / CELL), cj = Math.floor(z / CELL);
  for (var i = ci - 1; i <= ci + 1; i++) {
    var rowi = craterCells[wrapc(i)];
    for (var j = cj - 1; j <= cj + 1; j++) {
      var cc = rowi[wrapc(j)];
      if (!cc) continue;
      var dx = x - (i + cc.fx) * CELL, dz = z - (j + cc.fz) * CELL;
      var rad = cc.rad, d2 = dx * dx + dz * dz, reach = rad * 1.5;
      if (d2 > reach * reach) continue;
      var d = Math.sqrt(d2), depth = rad * 0.16;
      if (d < rad) { var q = 1 - (d / rad) * (d / rad); h -= depth * q * Math.sqrt(q); }
      var rim = (d - rad) / (rad * 0.22);
      h += depth * 0.5 * Math.exp(-rim * rim);
    }
  }
  return h;
}

function buildTerrainGeometry(seg) {
  var geo = new THREE.PlaneGeometry(PER, PER, seg, seg);
  geo.rotateX(-Math.PI / 2);
  var pos = geo.attributes.position;
  var colors = new Float32Array(pos.count * 3);
  for (var i = 0; i < pos.count; i++) {
    var x = pos.getX(i), z = pos.getZ(i);
    var h = terrainH(x, z);               // periodic: tile edges match neighbours
    pos.setY(i, h);
    var sp = hash2(Math.round(x * 7.3), Math.round(z * 7.7));
    var b = 0.9 + h * 0.22 + (sp - 0.5) * 0.18;
    b = Math.max(0.55, Math.min(1.35, b));
    colors[i * 3] = b; colors[i * 3 + 1] = b; colors[i * 3 + 2] = b * 1.02;
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

  /* ---------- post-processing: bloom + filmic grade + ACES output ---------- */
  var rtSamples = COARSE ? 0 : 4; // MSAA on desktop; rely on bloom softening on mobile
  var composer = new EffectComposer(renderer,
    new THREE.WebGLRenderTarget(2, 2, { type: THREE.HalfFloatType, samples: rtSamples }));
  composer.addPass(new RenderPass(scene, camera));
  var bloom = new UnrealBloomPass(new THREE.Vector2(2, 2), COARSE ? 0.55 : 0.75, 0.6, 0.9);
  composer.addPass(bloom);
  // filmic grade (linear HDR, before the ACES OutputPass): cool-shadow tint,
  // gentle saturation/contrast, vignette, faint grain
  var gradePass = new ShaderPass({
    uniforms: { tDiffuse: { value: null }, uTime: { value: 0 } },
    vertexShader: 'varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
    fragmentShader:
      'varying vec2 vUv; uniform sampler2D tDiffuse; uniform float uTime;' +
      'void main(){ vec3 c = texture2D(tDiffuse, vUv).rgb;' +
      ' float l = dot(c, vec3(0.2126,0.7152,0.0722));' +
      ' c = mix(vec3(l), c, 1.12);' +                                  // saturation
      ' c = (c - 0.5) * 1.06 + 0.5;' +                                 // contrast
      ' c += vec3(-0.012,0.004,0.022) * (1.0 - smoothstep(0.0,0.6,l));' + // teal shadow lift
      ' vec2 q = vUv - 0.5; c *= clamp(1.0 - dot(q,q) * 0.9, 0.0, 1.0);' + // vignette
      ' float g = fract(sin(dot(vUv, vec2(12.9898,78.233)) + uTime) * 43758.5453);' +
      ' c += (g - 0.5) * 0.014;' +                                     // grain
      ' gl_FragColor = vec4(max(c, 0.0), 1.0); }'
  });
  composer.addPass(gradePass);
  composer.addPass(new OutputPass()); // applies ACES tone map + sRGB last

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

  /* sky: stars + earth in a group that tracks the camera (kept at infinity) */
  var sky = new THREE.Group();
  scene.add(sky);
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
  sky.add(stars);

  var earth = new THREE.Mesh(new THREE.SphereGeometry(2.6, 48, 32), new THREE.MeshBasicMaterial({ map: earthTexture(), fog: false }));
  earth.position.set(-30, 10, -92); // earthrise, fixed bearing in the lunar sky
  sky.add(earth);
  var earthGlow = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTexture('rgba(120,170,255,0.5)', 'rgba(70,120,220,0.16)'), blending: THREE.AdditiveBlending, depthWrite: false, fog: false }));
  earthGlow.position.copy(earth.position);
  earthGlow.scale.setScalar(8);
  sky.add(earthGlow);

  /* scrolling world */
  var world = new THREE.Group();
  scene.add(world);

  var SEG = COARSE ? 60 : 96, GSTEP = PER / SEG;
  var terrainGeo = buildTerrainGeometry(SEG);
  // height of the RENDERED (tessellated) surface, bilinearly between grid verts,
  // so feet and rocks sit on what's drawn — not on sharper analytic crater dips
  function sampledH(x, z) {
    var gx = Math.floor(x / GSTEP) * GSTEP, gz = Math.floor(z / GSTEP) * GSTEP;
    var fx = (x - gx) / GSTEP, fz = (z - gz) / GSTEP;
    return terrainH(gx, gz) * (1 - fx) * (1 - fz) + terrainH(gx + GSTEP, gz) * fx * (1 - fz)
         + terrainH(gx, gz + GSTEP) * (1 - fx) * fz + terrainH(gx + GSTEP, gz + GSTEP) * fx * fz;
  }
  var terrainMat = new THREE.MeshStandardMaterial({ color: 0x595b60, roughness: 1, metalness: 0, vertexColors: true });
  var tiles = [];
  for (var ti = -1; ti <= 1; ti++) {
    for (var tj = -1; tj <= 1; tj++) {
      var tile = new THREE.Mesh(terrainGeo, terrainMat);
      tile.receiveShadow = true;
      tile.position.set(ti * PER, 0, tj * PER);
      world.add(tile);
      tiles.push(tile);
    }
  }
  // keep the 3x3 tile grid centred on the world point under the walker
  function snapTiles(cx, cz) {
    var sx = Math.round(cx / PER) * PER, sz = Math.round(cz / PER) * PER, k = 0;
    for (var a = -1; a <= 1; a++)
      for (var b = -1; b <= 1; b++) tiles[k++].position.set(sx + a * PER, 0, sz + b * PER);
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
  base.position.set(10, terrainH(10, -40), -40); // fixed lunar-base landmark, seated on terrain
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
  // rocks scattered across one period; each wraps to the cell nearest the walker
  var rocks = [];
  for (i = 0; i < 18; i++) {
    var rock = new THREE.Mesh(rockGeo, terrainMat);
    var rs = 0.12 + hash2(i, 11) * 0.6;
    rock.scale.setScalar(rs);
    var ox = (hash2(i, 19) - 0.5) * PER, oz = (hash2(i, 13) - 0.5) * PER;
    rock.receiveShadow = true;
    world.add(rock);
    rocks.push({ mesh: rock, ox: ox, oz: oz, y: sampledH(ox, oz) + rs * 0.25 });
  }
  function wrapRocks(cx, cz) {
    for (var r = 0; r < rocks.length; r++) {
      var rk = rocks[r];
      rk.mesh.position.set(
        Math.round((cx - rk.ox) / PER) * PER + rk.ox, rk.y,
        Math.round((cz - rk.oz) / PER) * PER + rk.oz);
    }
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
  // spawn at the walker's world-local footfall so the puff stays on the ground
  function puff(wx, wy, wz) {
    for (var pi = 0; pi < 3; pi++) {
      var d = dustPool[dustIdx];
      dustIdx = (dustIdx + 1) % dustPool.length;
      d.age = 0;
      d.life = 0.9 + Math.random() * 0.6;
      d.vy = 0.22 + Math.random() * 0.28;
      d.sp.position.set(wx + (Math.random() - 0.5) * 0.12, wy + 0.05, wz + (Math.random() - 0.5) * 0.12);
      d.sp.scale.setScalar(0.12 + Math.random() * 0.1);
      d.sp.visible = true;
    }
  }

  /* ---------- camera control ---------- */
  // yaw/pitch are a drag-orbit OFFSET on top of the auto chase angle
  var yaw = 0, pitch = 0.12, yawV = 0, dist = 5.7, lookY = 1.02, lookX = 0;
  var camHeading = 0, CAMSIDE = 0.5, camInit = false;
  var dragging = false, lx = 0, ly = 0, fired = false, activeId = null, lastMoveT = 0;
  var W = 0, H = 0;

  function applyQuality() {
    var pr = quality === 2 ? Math.min(DPR, MAXPR) : quality === 1 ? Math.min(DPR, 1.25) : 1;
    renderer.setPixelRatio(pr);
    composer.setPixelRatio(pr);
  }

  function resize() {
    W = window.innerWidth; H = window.innerHeight;
    renderer.setSize(W, H, false);
    composer.setSize(W, H);
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

  /* ---------- free-roam controller ---------- */
  // The robot stays at the scene origin facing +X; the WORLD group is moved and
  // rotated under it from the robot's virtual world pose, so the camera, sky and
  // shadow frustum never have to chase anything.
  var px = 0, pz = 0, heading = 0;   // robot's virtual world pose
  var spd = 0, trn = 0;              // eased speed (m/s) and turn rate (rad/s)
  var inFwd = 0, inTurn = 0, inActive = false;
  // seed gaitT to a double-support phase so the idle stance reads as a clean
  // stand; robotGY seeded to ground height (no startup pop / foot burial)
  var gaitT = 0.05 * GAIT.CYC, robotGY = sampledH(0, 0);
  var CRUISE = 0.5, MAXSPD = 1.7, MAXTRN = 1.0;
  function drive(fwd, turn, active) {
    inFwd = fwd < -1 ? -1 : (fwd > 1 ? 1 : fwd);
    inTurn = turn < -1 ? -1 : (turn > 1 ? 1 : turn);
    inActive = active === undefined ? (Math.abs(inFwd) > 0.04 || Math.abs(inTurn) > 0.04) : !!active;
    if (!fired) { fired = true; window.dispatchEvent(new CustomEvent('xvi-orbit')); }
    kick();
  }

  var raf = 0, last = performance.now(), t0 = last;
  var ema = 16, quality = 2, maxQ = 2, drops = 0; // pixel-ratio ladder state

  var prevPh = 0.0, sin = Math.sin, cos = Math.cos, moving = false;
  function pose() {
    if (!rig) return;
    var info = rig.update(gaitT);  // gaitT is frozen when idle -> static stance
    robot.position.y = robotGY + groundY0 + info.bob;
    if (moving) {
      var ph = info.phase, ch = cos(heading), sh = sin(heading);
      // footfall puffs at the walker's world-local feet (so they stay on the ground)
      if (ph < prevPh) puff(px - sh * 0.1, robotGY, pz + ch * 0.1);          // left heel strike
      if (prevPh < 0.5 && ph >= 0.5) puff(px + sh * 0.1, robotGY, pz - ch * 0.1); // right heel strike
    }
    prevPh = info.phase;
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

    // integrate the controller: ease toward the joystick target, advance the
    // virtual world pose, drive the gait by distance so the foot stays locked.
    // Reduced motion only disables the idle auto-cruise — the user can still
    // drive, and the robot still renders (it just doesn't move on its own).
    var tgtSpd = inActive ? inFwd * MAXSPD : (reduce ? 0 : CRUISE);
    var tgtTrn = inActive ? inTurn * MAXTRN : 0;
    spd += (tgtSpd - spd) * Math.min(1, dt * 4);
    trn += (tgtTrn - trn) * Math.min(1, dt * 6);
    heading += trn * dt;
    px += cos(heading) * spd * dt;
    pz += sin(heading) * spd * dt;
    // step cadence from forward speed AND turning, so the robot keeps stepping
    // (not rigidly pivoting) through and on the spot during turns. tsgn ramps
    // 1 -> -1 across spd in [0,-0.2] (continuous through 0): a standing turn
    // steps forward, only a real reverse steps backward
    var tsgn = spd > 0 ? 1 : Math.max(-1, 1 + spd / 0.1);
    var loco = spd + tsgn * Math.abs(trn) * 0.42;
    gaitT += loco * dt * (GAIT.STANCE * GAIT.CYC / GAIT.stepLen);
    robotGY += (sampledH(px, pz) - robotGY) * Math.min(1, dt * 6);
    moving = inActive || Math.abs(spd) > 0.02 || Math.abs(trn) > 0.02;

    // robot moves through a fixed world; terrain + props stream around it
    robot.position.x = px;
    robot.position.z = pz;
    robot.rotation.y = -heading;          // local +X faces the heading
    if (rig) rig.setBank(-trn * 0.16);    // bank into the turn
    snapTiles(px, pz);
    wrapRocks(px, pz);

    pose();

    // dust (rises and fades in world-local; stays on the ground as the robot moves on)
    for (var i = 0; i < dustPool.length; i++) {
      var d = dustPool[i];
      if (d.age > d.life) { d.sp.visible = false; continue; }
      d.age += dt;
      var u = d.age / d.life;
      d.sp.position.y += d.vy * dt;
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

    // third-person chase camera: smoothly trails behind the robot's heading
    // (eases around on turns), with drag (yaw/pitch) as an orbit offset
    var dh = heading - camHeading;
    while (dh > Math.PI) dh -= TAU; while (dh < -Math.PI) dh += TAU;
    camHeading += dh * Math.min(1, dt * 2.6);
    var tx = robot.position.x, ty = robotGY + lookY, tz = robot.position.z;
    var thc = camHeading + Math.PI + CAMSIDE + yaw; // behind + 3/4 offset + drag
    var cp = Math.cos(pitch), spv = Math.sin(pitch);
    // same trig basis as the robot's heading (cos->x, sin->z) so the camera
    // truly trails BEHIND the heading at every angle (not a mirrored one)
    var cpx = tx + Math.cos(thc) * dist * cp;
    var cpy = ty + 0.62 + spv * dist;
    var cpz = tz + Math.sin(thc) * dist * cp;
    var kf = camInit ? Math.min(1, dt * 7) : 1; // snap on the first frame
    camInit = true;
    camera.position.set(
      camera.position.x + (cpx - camera.position.x) * kf,
      camera.position.y + (cpy - camera.position.y) * kf,
      camera.position.z + (cpz - camera.position.z) * kf
    );
    // compose the robot off-centre on desktop (hero text sits on the left) by
    // aiming slightly to its side along the camera-right axis
    camera.lookAt(tx + Math.sin(thc) * lookX, ty, tz - Math.cos(thc) * lookX);

    // sky and shadow follow so stars/Earth stay at infinity and the shadow
    // frustum (fixed around its target) tracks the roaming robot
    sky.position.copy(camera.position);
    sun.position.set(tx + 14, robotGY + 15, tz + 13);
    sun.target.position.set(tx, robotGY, tz);

    gradePass.uniforms.uTime.value = t;
    composer.render();

    // Keep animating while moving/dragging (or always, when motion is allowed).
    // When idle under reduced motion, keep rendering for a short "settle" window
    // after the last kick so the robot/terrain reliably paint (mobile WebGL can
    // miss a single first frame) before the loop parks to save battery.
    if (!reduce || dragging || moving || now < settleUntil) {
      raf = requestAnimationFrame(frame);
    } else {
      raf = 0;
    }
  }

  var settleUntil = 0;
  function kick() {
    settleUntil = performance.now() + 1800; // render burst so a fresh frame paints
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

  // locomotion control surface for the on-screen joystick + keyboard (index.html)
  window.XVIControl = { drive: drive };
  // headless test hook: ?drive=<fwd>,<turn> applies a constant input
  try {
    var qd = (location.search.match(/[?&]drive=([^&]+)/) || [])[1];
    if (qd) { var p = decodeURIComponent(qd).split(','); drive(parseFloat(p[0]) || 0, parseFloat(p[1]) || 0, true); }
  } catch (e) {}

  return function () {
    window.XVIControl = null;
    cancelAnimationFrame(raf);
    canvas.removeEventListener('pointerdown', down);
    canvas.removeEventListener('webglcontextrestored', onRestore);
    window.removeEventListener('pointermove', move);
    window.removeEventListener('pointerup', up);
    window.removeEventListener('pointercancel', up);
    window.removeEventListener('resize', resize);
    composer.dispose();
    renderer.dispose();
  };
};

// the page's boot script may have given up polling on a slow connection —
// announce readiness so it can start the scene the moment the module lands
window.dispatchEvent(new Event('xvi-scene-ready'));
