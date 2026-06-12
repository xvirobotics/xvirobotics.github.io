/* XVI Robotics — lunar walk: a 3D point-cloud humanoid walking on the moon.
   Dependency-free canvas renderer with drag-to-orbit camera. */
(function () {
  'use strict';
  var TAU = Math.PI * 2;

  /* ---------- deterministic hash (stable crater field) ---------- */
  function hash2(i, j) {
    var n = Math.sin(i * 127.1 + j * 311.7) * 43758.5453;
    return n - Math.floor(n);
  }

  /* ---------- lunar terrain: gentle undulation + cratered cells,
     with a flattened corridor along z≈0 where the robot walks ---------- */
  var CELL = 110;
  function terrainH(x, z) {
    var h = 1.6 * Math.sin(x * 0.020 + 1.7) + 1.3 * Math.sin(z * 0.017 + 0.4) + 1.0 * Math.sin((x + z) * 0.011);
    var ci = Math.floor(x / CELL), cj = Math.floor(z / CELL);
    for (var i = ci - 1; i <= ci + 1; i++) {
      for (var j = cj - 1; j <= cj + 1; j++) {
        var r0 = hash2(i, j);
        if (r0 < 0.40) continue;
        var cx = (i + 0.18 + 0.64 * hash2(i + 7, j + 3)) * CELL;
        var cz = (j + 0.18 + 0.64 * hash2(i + 1, j + 9)) * CELL;
        var rad = 15 + 34 * hash2(i + 4, j + 6);
        var dx = x - cx, dz = z - cz;
        var d2 = dx * dx + dz * dz;
        var reach = rad * 1.5;
        if (d2 > reach * reach) continue;
        var d = Math.sqrt(d2);
        var depth = rad * 0.22;
        if (d < rad) {
          var q = 1 - (d / rad) * (d / rad);
          h -= depth * q * Math.sqrt(q);
        }
        var rim = (d - rad) / (rad * 0.22);
        h += depth * 0.62 * Math.exp(-rim * rim);
      }
    }
    var lat = Math.abs(z);
    if (lat < 64) {
      var k = lat < 22 ? 0 : (lat - 22) / 42;
      k = k * k * (3 - 2 * k);
      h *= 0.25 + 0.75 * k;
    }
    return h;
  }

  function ridgeH(x) {
    return 10 + 16 * Math.abs(Math.sin(x * 0.0082 + 1.3) + 0.55 * Math.sin(x * 0.021 + 2.6)) + 6 * Math.sin(x * 0.0034);
  }

  /* ---------- skeleton-attached point cloud ---------- */
  // bone ids
  var B_SPINE1 = 0, B_SPINE2 = 1, B_HIPBAR = 2, B_SHBAR = 3, B_NECK = 4,
      B_THIGH_L = 5, B_SHIN_L = 6, B_FOOT_L = 7,
      B_THIGH_R = 8, B_SHIN_R = 9, B_FOOT_R = 10,
      B_UARM_L = 11, B_FARM_L = 12, B_UARM_R = 13, B_FARM_R = 14,
      NBONES = 15;
  // sphere ids
  var S_HEAD = 0, S_HAND_L = 1, S_HAND_R = 2,
      S_SH_L = 3, S_SH_R = 4, S_KNEE_L = 5, S_KNEE_R = 6, NSPH = 7;

  function buildCloud() {
    var pts = [], spts = [];
    function roll() { var r = Math.random(); return r > 0.96 ? 2 : (r > 0.84 ? 1 : 0); }
    function capsule(b, n, r1, r2, su, sv) {
      su = su || 1; sv = sv || 1;
      for (var i = 0; i < n; i++) {
        var t = Math.random();
        var ang = Math.random() * TAU;
        var r = (r1 + (r2 - r1) * t) * (0.86 + Math.random() * 0.22);
        var cu = Math.cos(ang), cv = Math.sin(ang);
        pts.push({
          b: b, t: t, ox: cu * r * su, oy: cv * r * sv, nu: cu, nv: cv,
          ph: Math.random() * TAU, sp: 0.4 + Math.random() * 1.2,
          c: roll(), s: 0.7 + Math.random() * 0.9
        });
      }
    }
    function sphere(si, n, r) {
      for (var i = 0; i < n; i++) {
        var u = Math.random() * 2 - 1, th = Math.random() * TAU;
        var s = Math.sqrt(1 - u * u);
        var rr = r * (0.9 + Math.random() * 0.16);
        var sx = s * Math.cos(th), sy = u, sz = s * Math.sin(th);
        spts.push({
          si: si, sx: sx * rr, sy: sy * rr, sz: sz * rr, nx: sx, ny: sy, nz: sz,
          ph: Math.random() * TAU, sp: 0.4 + Math.random() * 1.2,
          c: roll(), s: 0.7 + Math.random() * 0.9
        });
      }
    }
    // Unitree H2 proportions, derived from the official URDF joint origins
    // (~98 scene units per meter): slim rounded torso, narrow shoulders,
    // short thighs with long slender shins, feet tracking inside the hips
    // (elliptical: wide across shoulders/hips = u(z) axis, slim front-back = v(x) axis)
    capsule(B_SPINE1, 340, 12.0, 10.5, 1.05, 0.62);
    capsule(B_SPINE2, 520, 10.5, 14.5, 1.15, 0.60);
    capsule(B_HIPBAR, 150, 6.5, 6.5);
    capsule(B_SHBAR, 170, 6.0, 6.0);
    capsule(B_NECK, 55, 3.8, 3.8);
    sphere(S_HEAD, 350, 8.6);
    sphere(S_SH_L, 70, 5.5);
    sphere(S_SH_R, 70, 5.5);
    sphere(S_KNEE_L, 55, 4.6);
    sphere(S_KNEE_R, 55, 4.6);
    var L = [B_THIGH_L, B_SHIN_L, B_FOOT_L, B_UARM_L, B_FARM_L],
        R = [B_THIGH_R, B_SHIN_R, B_FOOT_R, B_UARM_R, B_FARM_R];
    for (var s2 = 0; s2 < 2; s2++) {
      var g = s2 === 0 ? L : R;
      capsule(g[0], 220, 6.5, 5.0);
      capsule(g[1], 230, 4.8, 3.4);
      capsule(g[2], 100, 4.0, 3.2);
      capsule(g[3], 150, 4.6, 4.0);
      capsule(g[4], 140, 3.8, 3.2);
    }
    sphere(S_HAND_L, 70, 3.8);
    sphere(S_HAND_R, 70, 3.8);
    // visor: a steady bright cyan band wrapping the front of the helmet
    for (var vi = 0; vi < spts.length; vi++) {
      var vp = spts[vi];
      if (vp.si === S_HEAD && vp.nx > 0.42 && vp.ny > -0.42 && vp.ny < 0.12) {
        vp.c = 1;
        vp.s = Math.min(1.6, vp.s * 1.45);
        vp.vis = 1;
      }
    }
    return { pts: pts, spts: spts };
  }

  // approximate body silhouette half-width by height (for the scan ring)
  function silWidth(y) {
    if (y < 0) return 0;
    if (y < 12) return 17;
    if (y < 52) return 13;
    if (y < 84) return 15.5;
    if (y < 109) return 14.5;
    if (y < 146) return 18;
    if (y < 153) return 10;
    if (y < 172) return 9.5;
    return 0;
  }

  /* ---------- gait: slow lunar lope — long floaty strides, high lift ---------- */
  var CYC = 2.0, STEP = 34, LIFT = 15, STANCE = 0.5;
  // stance foot travels STEP over STANCE*CYC seconds; terrain must scroll at the
  // same speed or planted feet skate against the ground
  var SPEED = STEP / (STANCE * CYC);

  function footTrack(lp, out) {
    if (lp < STANCE) {
      var u = lp / STANCE;
      out.x = STEP / 2 - STEP * u;
      out.y = 0;
      out.air = 0;
    } else {
      var u2 = (lp - STANCE) / (1 - STANCE);
      var e = u2 * u2 * (3 - 2 * u2);
      out.x = -STEP / 2 + STEP * e;
      // flattened sine = hang time at the top of the swing (low gravity)
      var sl = Math.sin(Math.PI * u2);
      out.y = LIFT * Math.sqrt(sl > 0 ? sl : 0);
      out.air = 1;
    }
  }

  /* ---------- scene ---------- */
  window.XVIMoonWalk = function (canvas) {
    var ctx = canvas.getContext('2d');
    var W = 0, H = 0, cx = 0, cy = 0, f = 0, fs = 0, dpr = 1, dist = 365;
    var yaw = -0.34, pitch = -0.10, yawV = 0;
    var dragging = false, lx = 0, ly = 0, fired = false, activeId = null;
    var targY = 88;
    var reduce = false;
    try { reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches; } catch (err) {}

    var cloud = buildCloud();
    var pts = cloud.pts, spts = cloud.spts;
    var bones = [];
    for (var bi = 0; bi < NBONES; bi++) bones.push({ ax: 0, ay: 0, az: 0, ex: 0, ey: 0, ez: 0, ux: 0, uy: 0, uz: 0, vx: 0, vy: 0, vz: 0 });
    var sph = [];
    for (bi = 0; bi < NSPH; bi++) sph.push({ x: 0, y: 0, z: 0 });

    var stars = [], earthPts = [], dust = [], prints = [], meteor = null;
    var raf = 0, last = performance.now(), t0 = last;
    var scroll = 0, prevLp = [0, 0.5], nextMeteor = 5;
    var ema = 16, qStride = 1;

    for (var i = 0; i < 250; i++) {
      var u = Math.random() * 2 - 1, th = Math.random() * TAU;
      var sq = Math.sqrt(1 - u * u);
      stars.push({ x: sq * Math.cos(th), y: u * 0.75 + 0.18, z: sq * Math.sin(th), tw: Math.random() * TAU, sp: 0.4 + Math.random() * 1.6, b: 0.2 + Math.random() * 0.65, c: Math.random() > 0.92 ? 1 : 0 });
    }

    // Earth: point sphere with oceans / land / clouds and a day-night terminator
    for (i = 0; i < 600; i++) {
      var eu = Math.random() * 2 - 1, eth = Math.random() * TAU;
      var es = Math.sqrt(1 - eu * eu);
      var ex = es * Math.cos(eth), ey = eu, ez = es * Math.sin(eth);
      var land = Math.sin(ex * 3.2 + 1.1) + Math.sin(ey * 4.1 + 2.3) + Math.sin((ex + ez) * 2.6 + 0.7);
      var cloudy = Math.sin(ex * 5.1 + ey * 6.3 + 4.2) + Math.sin(ez * 7.7 + 1.9);
      var col = 4;
      if (cloudy > 1.25) col = 2; else if (land > 0.95) col = 5;
      earthPts.push({ x: ex, y: ey, z: ez, c: col });
    }
    var EDIR = { x: 0.58, y: 0.43, z: 0.88 }; // sky direction (rotates with camera yaw)

    /* fill-style cache: colors x 33 alpha buckets */
    var cols = [
      [156, 194, 220], // 0 steel blue
      [126, 224, 255], // 1 brand cyan
      [240, 250, 255], // 2 white
      [255, 110, 95],  // 3 unused (kept so earth/terrain indices 4-6 stay stable)
      [96, 165, 255],  // 4 earth ocean
      [125, 205, 170], // 5 earth land
      [186, 199, 212]  // 6 moon dust grey
    ];
    var fills = [];
    for (var c = 0; c < cols.length; c++) {
      fills[c] = [];
      for (var a = 0; a <= 32; a++) {
        fills[c][a] = 'rgba(' + cols[c][0] + ',' + cols[c][1] + ',' + cols[c][2] + ',' + (a / 32).toFixed(3) + ')';
      }
    }
    function fill(ci, alpha) {
      var b = alpha * 32;
      b = b < 0 ? 0 : (b > 32 ? 32 : b);
      return fills[ci][Math.round(b)];
    }

    /* terrain grid (heights cached; shifted by one column as the ground scrolls,
       fully recomputed only on resize or multi-step jumps). Extents scale with
       the viewport so ultrawide screens keep a full horizon. */
    var TSTEP = 9, TZ0 = -240, TZ1 = 240;
    var tRows = Math.floor((TZ1 - TZ0) / TSTEP) + 1;
    var TX0 = 0, TX1 = 0, tCols = 0;
    var tHeights = null, colEdge = null, rowEdge = null;
    var tSnap = null;
    function setupTerrain() {
      var half = Math.max(330, Math.ceil((W * 0.5) * 520 / f / TSTEP) * TSTEP + 60);
      TX0 = -half; TX1 = half + 130;
      tCols = Math.floor((TX1 - TX0) / TSTEP) + 1;
      tHeights = new Float32Array(tCols * tRows);
      colEdge = new Float32Array(tCols);
      rowEdge = new Float32Array(tRows);
      var FADE = 7; // cells of alpha falloff at the grid border
      for (var q = 0; q < tCols; q++) {
        var eq = Math.min(q, tCols - 1 - q) / FADE;
        colEdge[q] = eq > 1 ? 1 : eq;
      }
      for (var r = 0; r < tRows; r++) {
        var er = Math.min(r, tRows - 1 - r) / FADE;
        rowEdge[r] = er > 1 ? 1 : er;
      }
      tSnap = null;
    }
    function refreshTerrain(snap) {
      tSnap = snap;
      var n = 0;
      for (var r = 0; r < tRows; r++) {
        var wz = TZ0 + r * TSTEP;
        for (var q = 0; q < tCols; q++) {
          tHeights[n++] = terrainH(TX0 + q * TSTEP + snap, wz);
        }
      }
    }
    function shiftTerrain(snap) {
      // grid advanced exactly one column: shift left, recompute only the last
      // column of each row (the cells the flat copy corrupted)
      tSnap = snap;
      tHeights.copyWithin(0, 1);
      var lastX = TX0 + (tCols - 1) * TSTEP + snap;
      for (var r = 0; r < tRows; r++) {
        tHeights[r * tCols + tCols - 1] = terrainH(lastX, TZ0 + r * TSTEP);
      }
    }

    var LIT = { x: -0.42, y: 0.74, z: 0.34 }; // sun direction
    var ln = Math.sqrt(LIT.x * LIT.x + LIT.y * LIT.y + LIT.z * LIT.z);
    LIT.x /= ln; LIT.y /= ln; LIT.z /= ln;
    var VX = 0, VY = 0.25, VZ = 1; // toward-camera "headlight", updated per frame

    function resize() {
      dpr = Math.min(window.devicePixelRatio || 1, 2);
      W = window.innerWidth; H = window.innerHeight;
      canvas.width = W * dpr; canvas.height = H * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      cx = W * (W > 880 ? 0.63 : 0.5);
      // on phones the hero text is a compact block at the top, so frame the
      // walker lower and a bit smaller to keep the full body + ground visible
      cy = H * (W > 880 ? 0.52 : 0.63);
      f = H * 1.06;
      fs = Math.min(W, H) * 0.95;
      dist = W > 880 ? 365 : 500;
      setupTerrain();
      kick();
    }

    var cyw = 1, syw = 0, cp = 1, sp = 0;
    var P = [0, 0, 0];
    function project(x, y, z, out) {
      var rx = x * cyw + z * syw;
      var rz = -x * syw + z * cyw;
      var dy = y - targY;
      var ry = dy * cp - rz * sp;
      var rz2 = dy * sp + rz * cp;
      var zc = dist - rz2;
      if (zc < 40) return false;
      out[0] = cx + rx * f / zc;
      out[1] = cy - ry * f / zc;
      out[2] = zc;
      return true;
    }

    /* set a bone from joint a to joint b and build its local frame.
       refY: use (0,1,0) as the frame reference (for bones that stay roughly
       horizontal); otherwise (1,0,0) (for bones that stay roughly vertical).
       The reference is constant per bone — a direction-dependent branch here
       would make the u/v frame (and every attached point) snap 180 degrees
       whenever the gait crosses the branch threshold. */
    function setBone(id, ax, ay, az, bx, by, bz, refY) {
      var bn = bones[id];
      bn.ax = ax; bn.ay = ay; bn.az = az;
      var ex = bx - ax, ey = by - ay, ez = bz - az;
      bn.ex = ex; bn.ey = ey; bn.ez = ez;
      var len = Math.sqrt(ex * ex + ey * ey + ez * ez) || 1;
      var dx = ex / len, dy = ey / len, dz = ez / len;
      var rx, ry, rz;
      if (refY) { rx = 0; ry = 1; rz = 0; } else { rx = 1; ry = 0; rz = 0; }
      var ux = ry * dz - rz * dy, uy = rz * dx - rx * dz, uz = rx * dy - ry * dx;
      var ul = Math.sqrt(ux * ux + uy * uy + uz * uz) || 1;
      ux /= ul; uy /= ul; uz /= ul;
      bn.ux = ux; bn.uy = uy; bn.uz = uz;
      bn.vx = dy * uz - dz * uy;
      bn.vy = dz * ux - dx * uz;
      bn.vz = dx * uy - dy * ux;
    }

    var FOOT = { x: 0, y: 0, air: 0 };
    // H2 legs: short thigh, long shin (knee rides high); hips kept low enough
    // that the knees stay flexed like a real walking humanoid
    var THIGH = 31.5, SHIN = 48.7;

    function legIK(thighId, shinId, footId, kneeSi, hipX, hipY, hipZ, lp, sideZ, t) {
      footTrack(lp, FOOT);
      var ax = FOOT.x, ay = 7 + FOOT.y, az = sideZ;
      var dx = ax - hipX, dy = ay - hipY, dz = az - hipZ;
      var d = Math.sqrt(dx * dx + dy * dy + dz * dz);
      var maxD = THIGH + SHIN - 0.6;
      if (d > maxD) {
        var sc = maxD / d;
        ax = hipX + dx * sc; ay = hipY + dy * sc; az = hipZ + dz * sc;
        dx *= sc; dy *= sc; dz *= sc; d = maxD;
      }
      var ix = dx / d, iy = dy / d, iz = dz / d;
      // knee pole: forward (+x), orthogonalised against the leg axis
      var pd = ix; // dot((1,0,0), dir)
      var px2 = 1 - pd * ix, py2 = -pd * iy, pz2 = -pd * iz;
      var pl = Math.sqrt(px2 * px2 + py2 * py2 + pz2 * pz2) || 1;
      px2 /= pl; py2 /= pl; pz2 /= pl;
      var cosA = (THIGH * THIGH + d * d - SHIN * SHIN) / (2 * THIGH * d);
      cosA = cosA > 1 ? 1 : (cosA < -1 ? -1 : cosA);
      var sinA = Math.sqrt(1 - cosA * cosA);
      var kx = hipX + (ix * cosA + px2 * sinA) * THIGH;
      var ky = hipY + (iy * cosA + py2 * sinA) * THIGH;
      var kz = hipZ + (iz * cosA + pz2 * sinA) * THIGH;
      setBone(thighId, hipX, hipY, hipZ, kx, ky, kz);
      setBone(shinId, kx, ky, kz, ax, ay, az);
      sph[kneeSi].x = kx; sph[kneeSi].y = ky; sph[kneeSi].z = kz;
      // foot: ankle to toe (H2 has long flat feet), toe dips slightly during swing
      var toeDrop = FOOT.air * 2.6;
      setBone(footId, ax, ay - 2.2, az, ax + 17, Math.max(2.0, ay - 5.2 - toeDrop), az, true);
      return ax;
    }

    function armChain(uarmId, farmId, handSi, shSi, shX, shY, shZ, th, side) {
      // arms carried slightly out from the body with bent elbows — the
      // balance posture of someone walking in low gravity
      var th2 = th * 1.05 + 0.85;
      var ux0 = Math.sin(th), uy0 = -Math.cos(th), uz0 = side * 0.18;
      var l0 = Math.sqrt(ux0 * ux0 + uy0 * uy0 + uz0 * uz0);
      var ex = shX + 23 * ux0 / l0, eyy = shY + 23 * uy0 / l0, ez = shZ + 23 * uz0 / l0;
      var fx0 = Math.sin(th2), fy0 = -Math.cos(th2), fz0 = side * 0.08;
      var l1 = Math.sqrt(fx0 * fx0 + fy0 * fy0 + fz0 * fz0);
      var hx = ex + 26 * fx0 / l1, hy = eyy + 26 * fy0 / l1, hz = ez + 26 * fz0 / l1;
      setBone(uarmId, shX, shY, shZ, ex, eyy, ez);
      setBone(farmId, ex, eyy, ez, hx, hy, hz);
      sph[shSi].x = shX; sph[shSi].y = shY + 1.5; sph[shSi].z = shZ + side * 1.5;
      sph[handSi].x = hx; sph[handSi].y = hy; sph[handSi].z = hz;
    }

    var pose = { headX: 0, headY: 0, headZ: 0, chestX: 0, chestY: 0, chestZ: 0 };

    function computePose(t) {
      var phase = reduce ? 0.18 : (t / CYC) % 1;
      // lunar lope: pronounced vertical bounce, slight forward lean
      var bob = 3.4 * Math.sin(2 * TAU * phase + 0.7);
      var sway = -2.6 * Math.sin(TAU * phase);
      var lean = 6;
      var hipTw = 2.8 * Math.cos(TAU * phase);
      var shTw = -2.1 * Math.cos(TAU * phase);

      // H2 vertical stack from the URDF (hip pivot lowered so the knees stay
      // ~26 deg flexed): hips 81, waist ~107, chest/shoulders ~142, head ~161
      var hipY = 81 + bob;
      var waistX = lean * 0.45, waistY = hipY + 25.6, waistZ = sway * 0.8;
      var chestX = lean * 0.9, chestY = hipY + 61, chestZ = sway * 0.55;
      var headX = lean * 1.1, headY = hipY + 80, headZ = sway * 0.38;

      setBone(B_SPINE1, 0, hipY + 9, sway, waistX, waistY, waistZ);
      setBone(B_SPINE2, waistX, waistY, waistZ, chestX, chestY, chestZ);
      setBone(B_NECK, chestX * 1.05, hipY + 69, sway * 0.45, headX, headY - 6, headZ);
      setBone(B_HIPBAR, hipTw, hipY + 6.5, -13 + sway, -hipTw, hipY + 6.5, 13 + sway, true);
      setBone(B_SHBAR, chestX + shTw, hipY + 61, -16.5 + sway * 0.5, chestX - shTw, hipY + 61, 16.5 + sway * 0.5, true);
      sph[S_HEAD].x = headX; sph[S_HEAD].y = headY; sph[S_HEAD].z = headZ;

      var lpL = phase, lpR = (phase + 0.5) % 1;
      // H2: hip pivots sit wider (z ±13) than the feet track (z ±9.5)
      legIK(B_THIGH_L, B_SHIN_L, B_FOOT_L, S_KNEE_L, hipTw, hipY, -13 + sway * 0.6, lpL, -9.5, t);
      legIK(B_THIGH_R, B_SHIN_R, B_FOOT_R, S_KNEE_R, -hipTw, hipY, 13 + sway * 0.6, lpR, 9.5, t);

      var thL = -0.35 * Math.cos(TAU * phase);
      var thR = 0.35 * Math.cos(TAU * phase);
      armChain(B_UARM_L, B_FARM_L, S_HAND_L, S_SH_L, chestX + shTw, hipY + 59.5, -18 + sway * 0.5, thL, -1);
      armChain(B_UARM_R, B_FARM_R, S_HAND_R, S_SH_R, chestX - shTw, hipY + 59.5, 18 + sway * 0.5, thR, 1);

      pose.headX = headX; pose.headY = headY; pose.headZ = headZ;
      pose.chestX = chestX; pose.chestY = chestY; pose.chestZ = chestZ;

      // footfalls -> dust + footprints (left lands at phase 0, right at 0.5)
      if (!reduce) {
        var lps = [lpL, lpR];
        for (var leg = 0; leg < 2; leg++) {
          if (lps[leg] < prevLp[leg] - 0.5 || (prevLp[leg] > STANCE && lps[leg] < 0.1)) {
            plantFoot(leg === 0 ? -9.5 : 9.5);
          }
          prevLp[leg] = lps[leg];
        }
      }
    }

    function plantFoot(zSide) {
      prints.push({ wx: STEP / 2 + scroll, z: zSide, born: scroll });
      if (prints.length > 26) prints.shift();
      var n = 14 + (Math.random() * 7 | 0);
      for (var i = 0; i < n; i++) {
        if (dust.length > 220) dust.shift();
        dust.push({
          wx: STEP / 2 + scroll + (Math.random() * 9 - 4.5),
          y: 2 + Math.random() * 2.5,
          z: zSide + (Math.random() * 9 - 4.5),
          vx: -(4 + Math.random() * 11),
          vy: 8 + Math.random() * 14,
          vz: (Math.random() * 2 - 1) * 8,
          age: 0, life: 1.5 + Math.random() * 1.0
        });
      }
    }

    function frame(now) {
      var gap = now - last;
      var dt = Math.min(gap / 1000, 0.05);
      // clamp the EMA sample too — a multi-second gap after tab backgrounding
      // is a pause, not render cost, and must not trigger degraded quality
      ema += ((gap > 100 ? 100 : gap) - ema) * 0.05;
      last = now;
      var t = (now - t0) / 1000;
      var t1 = reduce ? 0 : t;
      if (qStride === 1 && ema > 40) qStride = 2;
      else if (qStride === 2 && ema < 24) qStride = 1;
      var stride = qStride;

      if (!dragging && !reduce) {
        yaw += yawV * dt;
        yawV *= Math.pow(0.12, dt);
      }
      if (!reduce) scroll += SPEED * dt;

      // idle motion is a bounded sway, not an unbounded spin (which would
      // expose the terrain grid's side edges); full 360 stays drag-reachable
      var yawR = yaw + (reduce ? 0 : 0.22 * Math.sin(t * 0.08));
      cyw = Math.cos(yawR); syw = Math.sin(yawR);
      cp = Math.cos(pitch); sp = Math.sin(pitch);
      var vl = Math.sqrt(syw * syw + 0.0625 + cyw * cyw);
      VX = -syw / vl; VY = 0.25 / vl; VZ = cyw / vl;

      computePose(t);

      ctx.fillStyle = '#04070b';
      ctx.fillRect(0, 0, W, H);

      /* stars */
      for (var si = 0; si < stars.length; si++) {
        var st = stars[si];
        var sx = st.x * cyw + st.z * syw;
        var sz = -st.x * syw + st.z * cyw;
        if (sz < 0.18) continue;
        var px = cx + (sx / sz) * fs;
        var py = cy - ((st.y + sz * sp * 0.4) / sz) * fs * 0.8;
        if (px < 0 || px > W || py < 0 || py > H) continue;
        var al = st.b * (0.45 + 0.55 * Math.sin(t1 * st.sp + st.tw));
        if (al <= 0.02) continue;
        ctx.fillStyle = fill(st.c, al * 0.55);
        ctx.fillRect(px, py, st.b > 0.7 ? 1.5 : 1, st.b > 0.7 ? 1.5 : 1);
      }

      /* meteor */
      if (!reduce) {
        if (!meteor && t > nextMeteor) {
          meteor = { x: W * (0.15 + Math.random() * 0.7), y: H * (0.04 + Math.random() * 0.2), a: 2.4 + Math.random() * 0.5, len: 130 + Math.random() * 110, t0: t, dur: 0.7 };
          nextMeteor = t + 6 + Math.random() * 9;
        }
        if (meteor) {
          var mu = (t - meteor.t0) / meteor.dur;
          if (mu >= 1) { meteor = null; }
          else {
            var fade = Math.sin(Math.PI * mu);
            var mx = meteor.x + Math.cos(meteor.a) * meteor.len * mu;
            var my = meteor.y - Math.sin(meteor.a) * meteor.len * mu;
            var grad = ctx.createLinearGradient(mx, my, mx - Math.cos(meteor.a) * 60, my + Math.sin(meteor.a) * 60);
            grad.addColorStop(0, 'rgba(240,250,255,' + (0.7 * fade).toFixed(3) + ')');
            grad.addColorStop(1, 'rgba(240,250,255,0)');
            ctx.strokeStyle = grad;
            ctx.lineWidth = 1.2;
            ctx.beginPath();
            ctx.moveTo(mx, my);
            ctx.lineTo(mx - Math.cos(meteor.a) * 60, my + Math.sin(meteor.a) * 60);
            ctx.stroke();
          }
        }
      }

      /* Earth */
      var edx = EDIR.x * cyw + EDIR.z * syw;
      var edz = -EDIR.x * syw + EDIR.z * cyw;
      if (edz > 0.25) {
        var epx = cx + (edx / edz) * fs;
        var epy = cy - ((EDIR.y + edz * sp * 0.4) / edz) * fs * 0.8;
        var eR = fs * 0.072;
        if (epx > -eR && epx < W + eR && epy > -eR && epy < H + eR) {
          var glow = ctx.createRadialGradient(epx, epy, eR * 0.3, epx, epy, eR * 1.9);
          glow.addColorStop(0, 'rgba(110,180,255,0.10)');
          glow.addColorStop(1, 'rgba(110,180,255,0)');
          ctx.fillStyle = glow;
          ctx.fillRect(epx - eR * 2, epy - eR * 2, eR * 4, eR * 4);
          var spin = t1 * 0.05;
          var cs = Math.cos(spin), sn = Math.sin(spin);
          for (var ei = 0; ei < earthPts.length; ei++) {
            var ep = earthPts[ei];
            var rx2 = ep.x * cs + ep.z * sn;
            var rz3 = -ep.x * sn + ep.z * cs;
            if (rz3 < 0.05) continue;
            var lit = rx2 * -0.62 + ep.y * 0.30 + rz3 * 0.72;
            var ea = lit > 0 ? 0.10 + 0.62 * lit : 0.05;
            ctx.fillStyle = fill(ep.c, ea);
            ctx.fillRect(epx + rx2 * eR, epy - ep.y * eR, 1.5, 1.5);
          }
        }
      }

      /* distant ridge (scrolls with the terrain) */
      ctx.strokeStyle = fill(0, 0.14);
      ctx.lineWidth = 1;
      ctx.beginPath();
      var started = false;
      for (var rxw = TX0 - 40; rxw <= TX1 + 40; rxw += 16) {
        var rwx = rxw + Math.floor(scroll / 16) * 16;
        if (project(rwx - scroll, ridgeH(rwx), -238, P)) {
          if (started) ctx.lineTo(P[0], P[1]); else { ctx.moveTo(P[0], P[1]); started = true; }
        } else started = false;
      }
      ctx.stroke();

      /* terrain dots */
      var snap = Math.floor(scroll / TSTEP) * TSTEP;
      if (snap !== tSnap) {
        if (tSnap !== null && snap - tSnap === TSTEP) shiftTerrain(snap);
        else refreshTerrain(snap);
      }
      var off = scroll - snap;
      var ck = snap / TSTEP; // anchors the stride-2 checkerboard to world space
      var n0 = 0;
      for (var tr = 0; tr < tRows; tr++) {
        var wz = TZ0 + tr * TSTEP;
        var re = rowEdge[tr];
        for (var tc = 0; tc < tCols; tc++, n0++) {
          if (stride === 2 && ((tr + tc + ck) & 1)) continue;
          var h = tHeights[n0];
          if (!project(TX0 + tc * TSTEP - off, h, wz, P)) continue;
          if (P[0] < -2 || P[0] > W + 2 || P[1] < -2 || P[1] > H + 2) continue;
          var depth = 1.65 - P[2] / 480;
          if (depth <= 0) continue;
          if (depth > 1) depth = 1;
          var ta = (0.22 + h * 0.034) * (0.30 + 0.70 * depth) * re * colEdge[tc];
          if (ta <= 0.02) continue;
          ctx.fillStyle = fill(h > 2.2 ? 6 : 0, ta > 0.44 ? 0.44 : ta);
          var tsz = depth > 0.72 ? 1.7 : 1.2;
          ctx.fillRect(P[0], P[1], tsz, tsz);
        }
      }

      /* footprints */
      for (var fp = 0; fp < prints.length; fp++) {
        var pr = prints[fp];
        var plx = pr.wx - scroll;
        if (plx < -290) continue;
        var pa = 0.30 * Math.exp(-(scroll - pr.born) / 220);
        if (pa < 0.02) continue;
        if (project(plx, 0.6, pr.z, P)) {
          if (P[0] < -4 || P[0] > W + 4 || P[1] < -4 || P[1] > H + 4) continue;
          ctx.fillStyle = fill(1, pa);
          ctx.fillRect(P[0] - 1.5, P[1] - 0.5, 3, 1.5);
          if (project(plx + 7, 0.6, pr.z, P)) ctx.fillRect(P[0] - 1.5, P[1] - 0.5, 3, 1.5);
        }
      }

      /* telemetry ring around the walker */
      ctx.strokeStyle = fill(1, 0.07);
      ctx.beginPath();
      started = false;
      for (var seg = 0; seg <= 72; seg++) {
        var an = seg / 72 * TAU;
        if (project(Math.cos(an) * 46, 0.8, Math.sin(an) * 46, P)) {
          if (started) ctx.lineTo(P[0], P[1]); else { ctx.moveTo(P[0], P[1]); started = true; }
        } else started = false;
      }
      ctx.stroke();
      ctx.strokeStyle = fill(1, 0.16);
      ctx.beginPath();
      for (var tk = 0; tk < 3; tk++) {
        var ta2 = t1 * 0.3 + tk / 3 * TAU;
        if (project(Math.cos(ta2) * 46, 0.8, Math.sin(ta2) * 46, P)) {
          ctx.moveTo(P[0], P[1]);
          if (project(Math.cos(ta2 + 0.13) * 46, 0.8, Math.sin(ta2 + 0.13) * 46, P)) ctx.lineTo(P[0], P[1]);
        }
      }
      ctx.stroke();

      /* soft contact shadow */
      if (project(2, 0, 0, P)) {
        var shR = 28 * f / P[2];
        var sg = ctx.createRadialGradient(P[0], P[1], 0, P[0], P[1], shR);
        sg.addColorStop(0, 'rgba(0,0,0,0.22)');
        sg.addColorStop(1, 'rgba(0,0,0,0)');
        ctx.save();
        ctx.translate(P[0], P[1]);
        ctx.scale(1, 0.32);
        ctx.translate(-P[0], -P[1]);
        ctx.fillStyle = sg;
        ctx.fillRect(P[0] - shR, P[1] - shR, shR * 2, shR * 2);
        ctx.restore();
      }

      /* hologram scan sweep (period 5.5s, sweep 1.7s) */
      var scanY = -999;
      if (!reduce) {
        var scT = t % 5.5;
        if (scT < 1.7) scanY = -8 + 200 * (scT / 1.7);
      }
      var sw = silWidth(scanY);
      if (sw > 0) {
        // the leaned figure carries its head forward — track the ring center
        var rcx = scanY > 150 ? 7 : 2;
        ctx.strokeStyle = fill(1, 0.15);
        ctx.beginPath();
        started = false;
        for (var sg2 = 0; sg2 <= 48; sg2++) {
          var sa = sg2 / 48 * TAU;
          if (project(Math.cos(sa) * sw + rcx, scanY, Math.sin(sa) * sw * 0.75, P)) {
            if (started) ctx.lineTo(P[0], P[1]); else { ctx.moveTo(P[0], P[1]); started = true; }
          } else started = false;
        }
        ctx.stroke();
      }

      /* the walker: bone-attached points with sun lighting */
      var np = pts.length;
      for (var pi = 0; pi < np; pi += stride) {
        var p = pts[pi];
        var bn = bones[p.b];
        var wx2 = bn.ax + bn.ex * p.t + bn.ux * p.ox + bn.vx * p.oy;
        var wy2 = bn.ay + bn.ey * p.t + bn.uy * p.ox + bn.vy * p.oy;
        var wz2 = bn.az + bn.ez * p.t + bn.uz * p.ox + bn.vz * p.oy;
        if (!project(wx2, wy2, wz2, P)) continue;
        var dep = 1.45 - P[2] / dist;
        if (dep <= 0) continue;
        if (dep > 1) dep = 1;
        var nx2 = bn.ux * p.nu + bn.vx * p.nv;
        var ny2 = bn.uy * p.nu + bn.vy * p.nv;
        var nz2 = bn.uz * p.nu + bn.vz * p.nv;
        var hdot = nx2 * VX + ny2 * VY + nz2 * VZ;
        var ldot = nx2 * LIT.x + ny2 * LIT.y + nz2 * LIT.z;
        var lt = 0.5 + (hdot > 0 ? 0.42 * hdot : 0) + (ldot > 0 ? 0.22 * ldot : 0);
        var dyS = (wy2 - scanY) / 8;
        var boost = dyS > -4 && dyS < 4 ? Math.exp(-dyS * dyS) * 0.5 : 0;
        var al2 = dep * (0.50 + 0.28 * Math.sin(t1 * p.sp * 1.3 + p.ph)) * lt + boost;
        if (al2 <= 0.02) continue;
        var size = p.s * (f / P[2]) * 0.56;
        if (size < 1) size = 1;
        ctx.fillStyle = fill(p.c, al2 > 0.95 ? 0.95 : al2);
        ctx.fillRect(P[0] - size / 2, P[1] - size / 2, size, size);
      }
      var ns = spts.length;
      for (pi = 0; pi < ns; pi += stride) {
        var p2 = spts[pi];
        var ce = sph[p2.si];
        var wx3 = ce.x + p2.sx, wy3 = ce.y + p2.sy, wz3 = ce.z + p2.sz;
        if (!project(wx3, wy3, wz3, P)) continue;
        var dep2 = 1.45 - P[2] / dist;
        if (dep2 <= 0) continue;
        if (dep2 > 1) dep2 = 1;
        var hdot2 = p2.nx * VX + p2.ny * VY + p2.nz * VZ;
        var ldot2 = p2.nx * LIT.x + p2.ny * LIT.y + p2.nz * LIT.z;
        var lt2 = 0.5 + (hdot2 > 0 ? 0.42 * hdot2 : 0) + (ldot2 > 0 ? 0.22 * ldot2 : 0);
        var dyS2 = (wy3 - scanY) / 8;
        var boost2 = dyS2 > -4 && dyS2 < 4 ? Math.exp(-dyS2 * dyS2) * 0.5 : 0;
        var al3 = p2.vis
          ? dep2 * (0.80 + 0.12 * Math.sin(t1 * 1.8 + p2.ph)) + boost2
          : dep2 * (0.50 + 0.28 * Math.sin(t1 * p2.sp * 1.3 + p2.ph)) * lt2 + boost2;
        if (al3 <= 0.02) continue;
        var size2 = p2.s * (f / P[2]) * 0.56;
        if (size2 < 1) size2 = 1;
        ctx.fillStyle = fill(p2.c, al3 > 0.95 ? 0.95 : al3);
        ctx.fillRect(P[0] - size2 / 2, P[1] - size2 / 2, size2, size2);
      }

      /* chest light (breathing) */
      var pulse = 0.35 + 0.4 * Math.sin(t1 * 2.4);
      if (project(pose.chestX + 9.5, pose.chestY - 6, pose.chestZ, P)) {
        var pr2 = 1.6 * f / P[2];
        var cg = ctx.createRadialGradient(P[0], P[1], 0, P[0], P[1], pr2 * 3);
        cg.addColorStop(0, 'rgba(200,240,255,' + (0.5 + pulse * 0.4).toFixed(3) + ')');
        cg.addColorStop(0.35, 'rgba(126,224,255,' + (0.16 + pulse * 0.2).toFixed(3) + ')');
        cg.addColorStop(1, 'rgba(126,224,255,0)');
        ctx.fillStyle = cg;
        ctx.fillRect(P[0] - pr2 * 3, P[1] - pr2 * 3, pr2 * 6, pr2 * 6);
      }
      /* low-gravity dust */
      for (var di = dust.length - 1; di >= 0; di--) {
        var dp = dust[di];
        dp.age += dt;
        if (dp.age > dp.life) { dust.splice(di, 1); continue; }
        dp.wx += dp.vx * dt;
        dp.z += dp.vz * dt;
        dp.vy -= 14 * dt;
        dp.y += dp.vy * dt;
        if (dp.y < 0.5) dp.y = 0.5;
        if (project(dp.wx - scroll, dp.y, dp.z, P)) {
          if (P[0] < -2 || P[0] > W + 2 || P[1] < -2 || P[1] > H + 2) continue;
          var da = 0.5 * (1 - dp.age / dp.life);
          ctx.fillStyle = fill(6, da);
          ctx.fillRect(P[0], P[1], 1.8, 1.8);
        }
      }

      // with reduced motion the image only changes while dragging — stop the
      // loop on an idle static frame instead of redrawing it at refresh rate
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

    var lastMoveT = 0;
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
      yaw += dx * 0.0062;
      if (edt > 0.0005) {
        // true angular velocity (rad/s) so flick inertia doesn't depend on
        // the pointer-event rate (60Hz vs 120Hz screens)
        var inst = (dx * 0.0062) / (edt < 0.004 ? 0.004 : edt);
        if (inst > 6) inst = 6; else if (inst < -6) inst = -6;
        yawV = yawV * 0.6 + inst * 0.4;
      }
      pitch = Math.max(-0.5, Math.min(0.32, pitch + dy * 0.004));
    }
    function up(e) {
      if (e && e.pointerId !== activeId) return;
      dragging = false;
      canvas.style.cursor = 'grab';
    }

    canvas.style.cursor = 'grab';
    // pinch-zoom stays native (WCAG 1.4.4); a second finger fires pointercancel,
    // which already ends the drag cleanly
    canvas.style.touchAction = 'pinch-zoom';
    canvas.addEventListener('pointerdown', down);
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
    window.addEventListener('pointercancel', up);
    window.addEventListener('resize', resize);
    resize();

    return function () {
      cancelAnimationFrame(raf);
      canvas.removeEventListener('pointerdown', down);
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
      window.removeEventListener('pointercancel', up);
      window.removeEventListener('resize', resize);
    };
  };
})();
