/* Unitree H2 — real URDF meshes assembled into an articulated rig and driven by
   a biomechanically-tuned walk. Geometry baked offline (vendor/h2.glb) from the
   official unitree_ros H2 description; kinematics from vendor/h2-kinematics.json. */
import * as THREE from 'three';
import { GLTFLoader } from './vendor/jsm/loaders/GLTFLoader.js';
import { DRACOLoader } from './vendor/jsm/loaders/DRACOLoader.js';

var TAU = Math.PI * 2;
var DEG = Math.PI / 180;

/* gait timing (shared with the scene so the treadmill locks to the stride) */
export var GAIT = { CYC: 1.25, STANCE: 0.6, stepLen: 0.46 };

function smooth(u) { return u * u * (3 - 2 * u); }

export function createH2Robot() {
  var loader = new GLTFLoader();
  // GLB is Draco-compressed (POSITION+indices) — decoder runs in a worker, off the
  // main thread, and outputs clean float geometry (no KHR_mesh_quantization node
  // transforms), so the rig-attach reset + computeVertexNormals below stay valid.
  var draco = new DRACOLoader();
  draco.setDecoderPath('vendor/jsm/libs/draco/gltf/');
  loader.setDRACOLoader(draco);
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
  // manifest: which GLB node belongs to which link, and which is the dark part
  var faceLink = kin.faceLink || 'head_yaw_link';
  var nodeInfo = {};       // nodeName -> { link, dark }
  (kin.meshNodes || []).forEach(function (mn) { nodeInfo[mn.node] = mn; });
  var meshByNode = {};
  gltf.scene.traverse(function (o) {
    if (o.isMesh && nodeInfo[o.name]) (meshByNode[o.name] = meshByNode[o.name] || []).push(o);
  });

  // factory two-tone: near-white shell everywhere, matte black only on the
  // real face-screen part the head DAE separates out
  var matShell = new THREE.MeshStandardMaterial({ color: 0xeef1f4, metalness: 0.28, roughness: 0.42 });
  var matFace = new THREE.MeshStandardMaterial({ color: 0x0a0d11, metalness: 0.5, roughness: 0.3, polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4 });
  function matForNode(mn) {
    return (mn.dark && mn.link === faceLink) ? matFace : matShell;
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

  // attach each node's mesh into its link group (vertices already in link frame)
  for (var nn in meshByNode) {
    var mn = nodeInfo[nn], grp = groups[mn.link], arr = meshByNode[nn];
    for (var m = 0; m < arr.length; m++) {
      var mesh = arr[m];
      mesh.position.set(0, 0, 0);
      mesh.quaternion.identity();
      mesh.scale.set(1, 1, 1);
      mesh.material = matForNode(mn);
      mesh.geometry.computeVertexNormals();
      mesh.castShadow = true;
      mesh.receiveShadow = false; // self-shadow on a small white robot isn't worth the per-fragment sample
      grp.add(mesh);
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

  /* ---- biomechanical walk ---- */
  // about the URDF +Y hip/knee axes a positive joint angle swings the leg
  // BACKWARD, so the forward stride needs negated hip/knee/ankle pitch
  var S = { hipPitch: -1, knee: 1, anklePitch: -1, foot: 1, hipRoll: 1 };
  // tunables (radians unless noted) — adjusted by watching a full-stride sheet
  var G = {
    standH: 0.92,        // stance hip height as a fraction of leg length
    bob: 0.026,          // vertical CoM oscillation
    swingLift: 0.022,    // ankle rise in swing
    tuck: 0.065,         // leg shortening in swing -> knee flexion (foot clearance)
    heel: 12 * DEG,      // dorsiflexion at heel strike (toe up)
    toe: 20 * DEG,       // plantarflexion at toe-off (heel up, push)
    hipAdduct: 2.2 * DEG,// legs track slightly under the body
    waistPitch: 5.5 * DEG,// forward lean
    waistYaw: 8 * DEG,   // thorax counter-rotation vs the legs
    waistRoll: 3 * DEG,  // lateral weight sway
    armSwing: 34 * DEG,
    armElbow: 15 * DEG,  // base elbow flexion
    armRoll: 7 * DEG
  };

  var FT = { x: 0, y: 0, pitch: 0, tuck: 0, air: 0 };
  function footTrack(lp) {
    var ST = GAIT.STANCE;
    if (lp < ST) {                                   // stance: planted, rolls heel->toe
      var u = lp / ST;
      FT.x = GAIT.stepLen / 2 - GAIT.stepLen * u;     // linear back == treadmill speed
      FT.y = 0; FT.tuck = 0; FT.air = 0;
      if (u < 0.15) FT.pitch = G.heel * (1 - u / 0.15);            // heel-strike dorsiflex fades
      else if (u > 0.6) FT.pitch = -G.toe * smooth((u - 0.6) / 0.4); // late-stance push-off
      else FT.pitch = 0;
    } else {                                         // swing: knee tucks up, foot reaches forward
      var u2 = (lp - ST) / (1 - ST);
      FT.x = -GAIT.stepLen / 2 + GAIT.stepLen * smooth(u2);
      var s = Math.sin(Math.PI * u2);
      FT.y = G.swingLift * s;
      FT.tuck = G.tuck * Math.sin(Math.PI * Math.min(1, u2 * 1.1)); // peak flex early-mid swing
      // ease the ankle from the toe-off push back up to heel-strike dorsiflexion,
      // so foot pitch is continuous across the toe-off boundary (no snap)
      FT.pitch = -G.toe + (G.heel + G.toe) * smooth(u2);
      FT.air = s;
    }
  }

  var IKO = { hip: 0, knee: 0, ankle: 0 };
  var UPD = { phase: 0, bob: 0 };  // reused per-frame return (no allocation in the loop)
  var bankRoll = 0;                // lean-into-turn, set by the scene
  function legIK(fx, fy, standH) {                    // 2-bone IK in the sagittal plane
    var depth = standH - fy;
    var d = Math.sqrt(fx * fx + depth * depth);
    var maxD = LEG - 0.003;
    if (d > maxD) d = maxD;
    var line = Math.atan2(fx, depth);
    var cosK = (THIGH * THIGH + d * d - SHIN * SHIN) / (2 * THIGH * d);
    cosK = cosK > 1 ? 1 : (cosK < -1 ? -1 : cosK);
    var thigh = line + Math.acos(cosK);
    var cosKnee = (THIGH * THIGH + SHIN * SHIN - d * d) / (2 * THIGH * SHIN);
    cosKnee = cosKnee > 1 ? 1 : (cosKnee < -1 ? -1 : cosKnee);
    var knee = Math.PI - Math.acos(cosKnee);
    IKO.hip = thigh; IKO.knee = knee; IKO.ankle = -(thigh - knee);
  }

  function leg(side, lp, standH) {
    footTrack(lp);
    var pre = side < 0 ? 'left_' : 'right_';
    legIK(FT.x, FT.y, standH - FT.tuck);             // tuck shortens the leg -> knee bends in swing
    setAngle(pre + 'hip_pitch_joint', S.hipPitch * IKO.hip);
    setAngle(pre + 'knee_joint', S.knee * IKO.knee);
    setAngle(pre + 'ankle_pitch_joint', S.anklePitch * IKO.ankle + S.foot * FT.pitch);
    setAngle(pre + 'hip_roll_joint', S.hipRoll * side * G.hipAdduct);
    return FT.air;
  }

  rig.update = function (t) {
    var phase = (t / GAIT.CYC) % 1;
    var w = TAU * phase;
    var bob = G.bob * Math.sin(2 * w + 0.7);
    // the scene applies bob to the whole rig; extend the legs by bob so the
    // planted stance foot stays pinned (the leg straightening IS the bob)
    var standH = rig.LEG * G.standH + bob;

    // trunk: forward lean, counter-rotation to the swinging legs, lateral sway
    setAngle('waist_pitch_joint', G.waistPitch);
    setAngle('waist_yaw_joint', G.waistYaw * Math.cos(w));
    setAngle('waist_roll_joint', G.waistRoll * Math.sin(w) + bankRoll); // + lean into turns

    leg(-1, phase, standH);
    leg(1, (phase + 0.5) % 1, standH);

    // arms swing opposite the legs; elbow bends a touch more on the forward swing
    var sw = G.armSwing * Math.cos(w);
    setAngle('left_shoulder_pitch_joint', sw);
    setAngle('right_shoulder_pitch_joint', -sw);
    setAngle('left_shoulder_roll_joint', G.armRoll);
    setAngle('right_shoulder_roll_joint', -G.armRoll);
    setAngle('left_elbow_joint', -(G.armElbow + Math.max(0, sw) * 0.5));
    setAngle('right_elbow_joint', -(G.armElbow + Math.max(0, -sw) * 0.5));

    // head holds forward and level while the thorax rotates under it
    setAngle('head_yaw_joint', -G.waistYaw * Math.cos(w) * 0.9);
    setAngle('head_pitch_joint', -3 * DEG);

    UPD.phase = phase; UPD.bob = bob;
    return UPD;
  };

  rig.G = G; rig.SIGN = S;
  rig.setBank = function (b) { bankRoll = b; };
  return rig;
}
