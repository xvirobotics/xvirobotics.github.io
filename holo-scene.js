/* XVI Robotics — interactive 3D holographic humanoid point cloud */
(function () {
  'use strict';
  var TAU = Math.PI * 2;

  function buildFigure() {
    var pts = [];
    function mk(x, y, z) {
      var roll = Math.random();
      return {
        x: x, y: y, z: z,
        ph: Math.random() * TAU,
        sp: 0.4 + Math.random() * 1.2,
        amp: 0.3 + Math.random() * 0.7,
        c: roll > 0.965 ? 2 : (roll > 0.86 ? 1 : 0),
        s: 0.7 + Math.random() * 0.9
      };
    }
    function capsule(a, b, r1, r2, n) {
      var ax = b[0] - a[0], ay = b[1] - a[1], az = b[2] - a[2];
      var len = Math.sqrt(ax * ax + ay * ay + az * az) || 1;
      ax /= len; ay /= len; az /= len;
      var ux, uy, uz;
      if (Math.abs(ay) < 0.9) { ux = az; uy = 0; uz = -ax; } else { ux = 1; uy = 0; uz = 0; }
      var ul = Math.sqrt(ux * ux + uy * uy + uz * uz) || 1;
      ux /= ul; uy /= ul; uz /= ul;
      var vx = ay * uz - az * uy, vy = az * ux - ax * uz, vz = ax * uy - ay * ux;
      for (var i = 0; i < n; i++) {
        var t = Math.random();
        var ang = Math.random() * TAU;
        var r = (r1 + (r2 - r1) * t) * (0.86 + Math.random() * 0.22);
        var px = a[0] + (b[0] - a[0]) * t, py = a[1] + (b[1] - a[1]) * t, pz = a[2] + (b[2] - a[2]) * t;
        var co = Math.cos(ang), si = Math.sin(ang);
        pts.push(mk(px + (ux * co + vx * si) * r, py + (uy * co + vy * si) * r, pz + (uz * co + vz * si) * r));
      }
    }
    function sphere(c, r, n) {
      for (var i = 0; i < n; i++) {
        var u = Math.random() * 2 - 1;
        var th = Math.random() * TAU;
        var s = Math.sqrt(1 - u * u);
        var rr = r * (0.9 + Math.random() * 0.16);
        pts.push(mk(c[0] + s * Math.cos(th) * rr, c[1] + u * rr, c[2] + s * Math.sin(th) * rr));
      }
    }
    function band(y0, y1, rx0, rx1, rz0, rz1, n) {
      for (var i = 0; i < n; i++) {
        var t = Math.random();
        var y = y0 + (y1 - y0) * t;
        var rx = rx0 + (rx1 - rx0) * t;
        var rz = rz0 + (rz1 - rz0) * t;
        var a = Math.random() * TAU;
        var j = 0.9 + Math.random() * 0.16;
        pts.push(mk(Math.cos(a) * rx * j, y, Math.sin(a) * rz * j));
      }
    }
    sphere([0, 165, 0], 10.5, 420);
    capsule([0, 149, 0], [0, 157, 0], 4.5, 4.5, 60);
    band(148, 118, 21, 15, 11, 9.5, 680);
    band(118, 96, 15, 17, 9.5, 9.5, 360);
    band(96, 84, 17, 13, 9.5, 8, 240);
    capsule([-21, 143, 0], [21, 143, 0], 8, 8, 220);
    var sgn = [-1, 1];
    for (var s = 0; s < 2; s++) {
      var g = sgn[s];
      capsule([g * 25, 141, 0], [g * 29, 113, 3], 6, 5, 240);
      capsule([g * 29, 113, 3], [g * 31, 87, 9], 4.8, 4, 220);
      sphere([g * 31, 79, 11], 5, 90);
      capsule([g * 10, 86, 0], [g * 12, 45, 2], 7.5, 5.8, 300);
      capsule([g * 12, 45, 2], [g * 13, 8, 0], 5.4, 4.4, 260);
      capsule([g * 13, 5, 2], [g * 13, 4, 16], 4.4, 3.4, 80);
    }
    return pts;
  }

  function silWidth(y) {
    if (y < 12) return 17;
    if (y < 48) return 15;
    if (y < 86) return 17;
    if (y < 122) return 19;
    if (y < 152) return 25;
    if (y < 176) return 12.5;
    return 0;
  }

  window.XVIHoloScene = function (canvas) {
    var ctx = canvas.getContext('2d');
    var W = 0, H = 0, cx = 0, cy = 0, f = 0, fs = 0, dpr = 1;
    var yaw = -0.38, pitch = -0.07, yawV = 0;
    var dragging = false, lx = 0, ly = 0, fired = false, activeId = null;
    var reduce = false;
    try { reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches; } catch (err) {}
    var dist = 335, targY = 88;
    var pts = buildFigure();
    var stars = [], motes = [];
    var raf = 0, last = performance.now();

    for (var i = 0; i < 240; i++) {
      var u = Math.random() * 2 - 1, th = Math.random() * TAU;
      var sq = Math.sqrt(1 - u * u);
      stars.push({ x: sq * Math.cos(th), y: u * 0.75 + 0.18, z: sq * Math.sin(th), tw: Math.random() * TAU, sp: 0.4 + Math.random() * 1.6, b: 0.2 + Math.random() * 0.65 });
    }
    for (i = 0; i < 90; i++) {
      motes.push({ x: (Math.random() * 2 - 1) * 160, y: Math.random() * 210, z: (Math.random() * 2 - 1) * 160, ph: Math.random() * TAU, sp: 0.2 + Math.random() * 0.5 });
    }

    // quantized fill-style cache: 3 colors x 33 alpha buckets
    var cols = [[156, 194, 220], [126, 224, 255], [240, 250, 255]];
    var fills = [];
    for (var c = 0; c < 3; c++) {
      fills[c] = [];
      for (var a = 0; a <= 32; a++) {
        fills[c][a] = 'rgba(' + cols[c][0] + ',' + cols[c][1] + ',' + cols[c][2] + ',' + (a / 32).toFixed(3) + ')';
      }
    }
    function fill(c, alpha) {
      var b = Math.max(0, Math.min(32, Math.round(alpha * 32)));
      return fills[c][b];
    }

    function resize() {
      dpr = Math.min(window.devicePixelRatio || 1, 2);
      W = window.innerWidth; H = window.innerHeight;
      canvas.width = W * dpr; canvas.height = H * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      cx = W * (W > 880 ? 0.665 : 0.5);
      cy = H * 0.54;
      f = H * 1.06;
      fs = Math.min(W, H) * 0.95;
    }

    function project(x, y, z, cyw, syw, cp, sp, out) {
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

    var P = [0, 0, 0];

    function frame(now) {
      var dt = Math.min((now - last) / 1000, 0.05);
      last = now;

      if (!dragging && !reduce) {
        yaw += (0.11 + yawV) * dt;
        yawV *= Math.pow(0.12, dt);
      }

      var cyw = Math.cos(yaw), syw = Math.sin(yaw);
      var cp = Math.cos(pitch), sp = Math.sin(pitch);
      var t1 = reduce ? 0 : now * 0.001;

      ctx.fillStyle = '#04070b';
      ctx.fillRect(0, 0, W, H);

      // stars (infinite sphere, yaw-only parallax)
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
        ctx.fillStyle = fill(0, al * 0.55);
        ctx.fillRect(px, py, st.b > 0.7 ? 1.5 : 1, st.b > 0.7 ? 1.5 : 1);
      }

      // polar floor grid
      var rings = [45, 80, 115, 150];
      for (var ri = 0; ri < rings.length; ri++) {
        var r = rings[ri];
        ctx.strokeStyle = fill(0, 0.1 - ri * 0.018);
        ctx.lineWidth = 1;
        ctx.beginPath();
        var started = false;
        for (var seg = 0; seg <= 96; seg++) {
          var an = seg / 96 * TAU;
          if (project(Math.cos(an) * r, 0, Math.sin(an) * r, cyw, syw, cp, sp, P)) {
            if (started) ctx.lineTo(P[0], P[1]); else { ctx.moveTo(P[0], P[1]); started = true; }
          }
        }
        ctx.stroke();
      }
      ctx.strokeStyle = fill(0, 0.09);
      ctx.beginPath();
      for (var tk = 0; tk < 24; tk++) {
        var ta = tk / 24 * TAU;
        if (project(Math.cos(ta) * 150, 0, Math.sin(ta) * 150, cyw, syw, cp, sp, P)) {
          ctx.moveTo(P[0], P[1]);
          if (project(Math.cos(ta) * 157, 0, Math.sin(ta) * 157, cyw, syw, cp, sp, P)) ctx.lineTo(P[0], P[1]);
        }
      }
      ctx.stroke();

      // drifting motes
      for (var mi = 0; mi < motes.length; mi++) {
        var m = motes[mi];
        var my = m.y + Math.sin(t1 * m.sp + m.ph) * 6;
        if (project(m.x, my, m.z, cyw, syw, cp, sp, P)) {
          ctx.fillStyle = fill(0, 0.05 + 0.07 * Math.sin(t1 * m.sp * 1.7 + m.ph));
          ctx.fillRect(P[0], P[1], 1, 1);
        }
      }

      // scan band sweeping up the figure
      var scanY = reduce ? -999 : ((now * 0.00022) % 1) * 200 - 8;

      // scan ring
      var sw = silWidth(scanY);
      if (sw > 0 && !reduce) {
        ctx.strokeStyle = fill(1, 0.16);
        ctx.beginPath();
        var st2 = false;
        for (var sg = 0; sg <= 48; sg++) {
          var sa = sg / 48 * TAU;
          if (project(Math.cos(sa) * sw, scanY, Math.sin(sa) * sw * 0.7, cyw, syw, cp, sp, P)) {
            if (st2) ctx.lineTo(P[0], P[1]); else { ctx.moveTo(P[0], P[1]); st2 = true; }
          }
        }
        ctx.stroke();
      }

      // figure point cloud + floor reflection
      var n = pts.length;
      for (var pi = 0; pi < n; pi++) {
        var p = pts[pi];
        var w1 = Math.sin(t1 * p.sp + p.ph) * p.amp;
        var w2 = Math.sin(t1 * p.sp * 0.8 + p.ph * 1.7) * p.amp * 0.5;
        var x = p.x + w1 * 0.6, y = p.y + w2, z = p.z + w1 * 0.4;
        if (!project(x, y, z, cyw, syw, cp, sp, P)) continue;
        var depth = Math.max(0, Math.min(1, 1.45 - P[2] / dist));
        var dyS = (y - scanY) / 8;
        var boost = Math.exp(-dyS * dyS) * 0.55;
        var al = depth * (0.42 + 0.3 * Math.sin(t1 * p.sp * 1.3 + p.ph)) + boost;
        if (al <= 0.02) continue;
        var size = Math.max(1, p.s * (f / P[2]) * 0.52);
        ctx.fillStyle = fill(p.c, Math.min(0.95, al));
        ctx.fillRect(P[0] - size / 2, P[1] - size / 2, size, size);
        if (pi % 3 === 0) {
          if (project(x, -y * 0.85, z, cyw, syw, cp, sp, P)) {
            ctx.fillStyle = fill(p.c, Math.min(0.95, al) * 0.1);
            ctx.fillRect(P[0] - size / 2, P[1] - size / 2, size, size);
          }
        }
      }

      raf = requestAnimationFrame(frame);
    }

    function down(e) {
      if (e.button !== 0 || !e.isPrimary) return;
      dragging = true; activeId = e.pointerId; lx = e.clientX; ly = e.clientY;
      canvas.style.cursor = 'grabbing';
      try { canvas.setPointerCapture(e.pointerId); } catch (err) {}
      if (!fired) { fired = true; window.dispatchEvent(new CustomEvent('xvi-orbit')); }
    }
    function move(e) {
      if (!dragging || e.pointerId !== activeId) return;
      var dx = e.clientX - lx, dy = e.clientY - ly;
      lx = e.clientX; ly = e.clientY;
      yaw += dx * 0.0062;
      yawV = dx * 0.34;
      pitch = Math.max(-0.55, Math.min(0.34, pitch + dy * 0.004));
    }
    function up(e) {
      if (e && e.pointerId !== activeId) return;
      dragging = false;
      canvas.style.cursor = 'grab';
    }

    canvas.style.cursor = 'grab';
    canvas.style.touchAction = 'none';
    canvas.addEventListener('pointerdown', down);
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
    window.addEventListener('pointercancel', up);
    window.addEventListener('resize', resize);
    resize();
    raf = requestAnimationFrame(frame);

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
