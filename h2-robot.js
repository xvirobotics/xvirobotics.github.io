/* Unitree H2 — real URDF meshes assembled into an articulated rig and driven by
   a biomechanically-tuned walk. Geometry baked offline (vendor/h2.glb) from the
   official unitree_ros H2 description; kinematics from vendor/h2-kinematics.json. */
import * as THREE from 'three';
import { GLTFLoader } from './vendor/jsm/loaders/GLTFLoader.js';

var TAU = Math.PI * 2;
var DEG = Math.PI / 180;

/* gait timing (shared with the scene so the treadmill locks to the stride) */
export var GAIT = { CYC: 1.45, STANCE: 0.62, stepLen: 0.36 };

function smooth(u) { return u * u * (3 - 2 * u); }

export function createH2Robot() {
  var loader = new GLTFLoader();
  return Promise.all([
    new Promise(function (res, rej) { loader.load('vendor/h2.glb', res, undefined, rej); }),
    fetch('vendor/h2-kinematics.json').then(function (r) { return r.json(); })
  ]).then(function (out) { return buildRig(out[0], out[1]); });
}

function buildRig(gltf, kin) {
  // collect meshes by the link name they belong to
  var linkNames = {};
  for (var ln in kin.links) linkNames[ln] = true;
  var meshByLink = {};
  gltf.scene.traverse(function (o) {
    if (!o.isMesh) return;
    var n = o;
    while (n && !(n.name in linkNames)) n = n.parent;
    if (n) (meshByLink[n.name] = meshByLink[n.name] || []).push(o);
  });

  // materials approximating H2's white shell with dark actuators
  var matShell = new THREE.MeshStandardMaterial({ color: 0xeef1f4, metalness: 0.28, roughness: 0.42 });
  var matDark = new THREE.MeshStandardMaterial({ color: 0x1b1f24, metalness: 0.72, roughness: 0.46 });
  function matFor(link) {
    return matShell; // unified white shell across the whole body
  }

  // one group per link; parent per the joint tree
  var groups = {};
  for (var l in kin.links) {
    var g = new THREE.Group();
    g.name = l;
    groups[l] = g;
  }
  var joints = {};
  var q = new THREE.Quaternion();
  for (var i = 0; i < kin.joints.length; i++) {
    var j = kin.joints[i];
    var cg = groups[j.child];
    cg.position.set(j.origin_xyz[0], j.origin_xyz[1], j.origin_xyz[2]);
    var oq = j.origin_quat;
    cg.quaternion.set(oq[0], oq[1], oq[2], oq[3]);
    groups[j.parent].add(cg);
    joints[j.name] = {
      group: cg,
      axis: new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
      base: cg.quaternion.clone(),
      type: j.type
    };
  }

  // attach each link's mesh into its link group (vertices already in link frame)
  for (var lk in meshByLink) {
    var arr = meshByLink[lk];
    for (var m = 0; m < arr.length; m++) {
      var mesh = arr[m];
      mesh.position.set(0, 0, 0);
      mesh.quaternion.identity();
      mesh.scale.set(1, 1, 1);
      mesh.material = matFor(lk);
      if (!mesh.geometry.getAttribute('normal')) mesh.geometry.computeVertexNormals();
      mesh.castShadow = true;
      mesh.receiveShadow = false; // self-shadow on a small white robot isn't worth the per-fragment sample
      groups[lk].add(mesh);
    }
  }

  var root = groups[kin.root];

  // leg segment lengths from the URDF origins (for IK)
  function originOf(name) {
    for (var k = 0; k < kin.joints.length; k++) if (kin.joints[k].name === name) return kin.joints[k].origin_xyz;
    return [0, 0, 0];
  }
  function chainLen() {
    var args = Array.prototype.slice.call(arguments), L = 0;
    for (var k = 0; k < args.length; k++) { var o = originOf(args[k]); L += Math.sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]); }
    return L;
  }
  var THIGH = chainLen('left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint');
  var SHIN = chainLen('left_ankle_roll_joint', 'left_ankle_pitch_joint');
  var LEG = THIGH + SHIN;

  var tmpQ = new THREE.Quaternion();
  function setAngle(name, theta) {
    var r = joints[name];
    if (!r) return;
    tmpQ.setFromAxisAngle(r.axis, theta);
    r.group.quaternion.copy(r.base).multiply(tmpQ);
  }

  // accent attach points (head/torso links) for the scene's brand glow
  var headG = groups['head_yaw_link'] || groups['head_pitch_link'] || groups['torso_link'];
  var torsoG = groups['torso_link'];

  var rig = {
    root: root,
    groups: groups,
    joints: joints,
    setAngle: setAngle,
    THIGH: THIGH, SHIN: SHIN, LEG: LEG,
    headGroup: headG, torsoGroup: torsoG
  };

  /* ---- biomechanical walk: drive every joint from one phase ---- */
  // sign conventions resolved against the H2 URDF axes (tuned visually)
  // about the URDF +Y hip/knee axes, a positive joint angle swings the leg
  // BACKWARD, so the forward stride needs negated hip/knee/ankle pitch
  var S = { hipPitch: -1, knee: 1, anklePitch: -1, hipRoll: 1, shoulder: 1 };
  // arm carriage (deg) — tuned visually; pitch0=0 hangs straight down
  var ARM = { pitch0: 0, swing: 18, roll: 6, elbow: 0 };

  function footTrack(lp, out) {
    var ST = GAIT.STANCE;
    if (lp < ST) {                         // stance: foot moves back, planted
      var u = lp / ST;
      out.x = GAIT.stepLen / 2 - GAIT.stepLen * u;
      out.y = 0; out.air = 0;
    } else {                               // swing: foot lifts and reaches forward
      var u2 = (lp - ST) / (1 - ST);
      out.x = -GAIT.stepLen / 2 + GAIT.stepLen * smooth(u2);
      var s = Math.sin(Math.PI * u2);
      out.y = 0.135 * Math.sqrt(s > 0 ? s : 0);  // low-g hang
      out.air = s;
    }
  }

  var FT = { x: 0, y: 0, air: 0 };
  // returns {hip,knee,ankle} sagittal joint angles to put the foot at (fx, fy)
  function legIK(fx, fy, standH) {
    footHelper(fx, fy, standH);
    return footHelper.out;
  }
  function footHelper(fx, fy, standH) {
    var dx = fx, dz = -(standH - fy);        // hip at origin; down is -z(up)
    var d = Math.sqrt(dx * dx + dz * dz);
    var maxD = LEG - 0.003;
    if (d > maxD) { d = maxD; }
    // angle of hip->foot line from straight down
    var line = Math.atan2(dx, standH - fy);
    var cosK = (THIGH * THIGH + d * d - SHIN * SHIN) / (2 * THIGH * d);
    cosK = cosK > 1 ? 1 : (cosK < -1 ? -1 : cosK);
    var hipFromLine = Math.acos(cosK);
    var thigh = line + hipFromLine;          // thigh pitch fwd from vertical
    var cosKnee = (THIGH * THIGH + SHIN * SHIN - d * d) / (2 * THIGH * SHIN);
    cosKnee = cosKnee > 1 ? 1 : (cosKnee < -1 ? -1 : cosKnee);
    var knee = Math.PI - Math.acos(cosKnee); // interior flex (0 = straight)
    var shinPitch = thigh - knee;
    footHelper.out = { hip: thigh, knee: knee, ankle: -shinPitch };
  }
  footHelper.out = { hip: 0, knee: 0, ankle: 0 };

  function leg(side, lp, standH, list) {
    footTrack(lp, FT);
    var pre = side < 0 ? 'left_' : 'right_';
    var a = legIK(FT.x, FT.y, standH);
    setAngle(pre + 'hip_pitch_joint', S.hipPitch * a.hip);
    setAngle(pre + 'knee_joint', S.knee * a.knee);
    // keep the sole roughly flat, add a toe-off push during late stance
    var toe = (lp < GAIT.STANCE && lp > GAIT.STANCE * 0.7) ? (lp - GAIT.STANCE * 0.7) / (GAIT.STANCE * 0.3) * 0.35 : 0;
    setAngle(pre + 'ankle_pitch_joint', S.anklePitch * (a.ankle + toe));
    setAngle(pre + 'hip_roll_joint', S.hipRoll * side * list);
    return FT.air;
  }

  rig.update = function (t) {
    var phase = (t / GAIT.CYC) % 1;
    var bob = 0.028 * Math.sin(2 * TAU * phase + 0.7);
    // the body bob is applied to the whole rig by the scene; extend the legs by
    // the same bob so the planted stance foot stays pinned to the ground (the
    // stance leg straightening IS the bob, instead of the foot sliding vertically)
    var standH = rig.LEG * 0.93 + bob;
    var sway = 2.0 * DEG * Math.sin(TAU * phase);    // pelvis frontal sway
    var pelvisYaw = 4 * DEG * Math.cos(TAU * phase);

    // pelvis articulation goes through the waist + hip joints; the holder owns
    // the Z-up->Y-up orientation, so the root stays identity here
    setAngle('waist_yaw_joint', -pelvisYaw * 1.4);
    setAngle('waist_pitch_joint', 6 * DEG);          // slight forward stoop
    setAngle('waist_roll_joint', sway * 0.5);

    var airL = leg(-1, phase, standH, sway);
    var airR = leg(1, (phase + 0.5) % 1, standH, sway);

    // arms swing opposite the legs, slight abduction, fixed elbow bend
    var sw = ARM.swing * DEG * Math.cos(TAU * phase);
    setAngle('left_shoulder_pitch_joint', ARM.pitch0 * DEG + sw);
    setAngle('right_shoulder_pitch_joint', ARM.pitch0 * DEG - sw);
    setAngle('left_shoulder_roll_joint', ARM.roll * DEG);
    setAngle('right_shoulder_roll_joint', -ARM.roll * DEG);
    setAngle('left_elbow_joint', ARM.elbow * DEG);
    setAngle('right_elbow_joint', ARM.elbow * DEG);

    setAngle('head_pitch_joint', -4 * DEG);          // gaze up toward the horizon

    return { phase: phase, bob: bob, pelvisYaw: pelvisYaw, footAirL: airL, footAirR: airR, standH: standH };
  };

  // expose sign table so the scene can flip conventions during tuning
  rig.SIGN = S;
  return rig;
}
