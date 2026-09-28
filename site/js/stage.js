// tangent: the stage: the part, turning under the light.
//
// The part is tools/site_model.cpp's output: triangles, normals and edges from
// the kernel, and for every vertex the operation that made its face. The stage
// sits fixed behind the page. It holds the part beside the hero, keeps it
// beside the features while they scroll past (lighting up the faces each one
// is about), and fades it out before the rest of the page.
import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';

const DEG = Math.PI / 180;
const clamp01 = x => Math.min(1, Math.max(0, x));
const smooth = x => { x = clamp01(x); return x * x * (3 - 2 * x); };
const damp = (a, b, rate, dt) => b + (a - b) * Math.exp(-rate * dt);

function token(name) {
  return new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue(name).trim());
}

// ── The part ────────────────────────────────────────────────────────────
async function loadPart(url) {
  const buf = await (await fetch(url)).arrayBuffer();
  const dv = new DataView(buf);
  if (String.fromCharCode(...new Uint8Array(buf, 0, 4)) !== 'TGM1') throw new Error('not a TGM1 file');
  const nV = dv.getUint32(4, true), nT = dv.getUint32(8, true), nE = dv.getUint32(12, true);
  let at = 40;
  // Sections are copied out rather than viewed: the byte-wide groups leave
  // the indices after them unaligned.
  const take = (Type, n, size) => { const a = new Type(buf.slice(at, at + n * size)); at += n * size; return a; };
  const positions = take(Float32Array, nV * 3, 4);
  const normals   = take(Int16Array,   nV * 3, 2);
  const groups    = take(Uint8Array,   nV,     1);
  const indices   = take(Uint32Array,  nT * 3, 4);
  const edges     = take(Float32Array, nE * 6, 4);

  const surface = new THREE.BufferGeometry();
  surface.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  surface.setAttribute('normal', new THREE.BufferAttribute(normals, 3, true));
  surface.setAttribute('aGroup', new THREE.BufferAttribute(Float32Array.from(groups), 1));
  surface.setIndex(new THREE.BufferAttribute(indices, 1));

  const lines = new THREE.BufferGeometry();
  lines.setAttribute('position', new THREE.BufferAttribute(edges, 3));
  return { surface, lines };
}

// The part's surface, with each operation's faces able to take the tint.
function partMaterial(colour, uniforms) {
  const m = new THREE.MeshPhysicalMaterial({
    color: colour, roughness: 0.36, metalness: 0.0,
    clearcoat: 0.4, clearcoatRoughness: 0.28,
    polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
  });
  m.onBeforeCompile = shader => {
    Object.assign(shader.uniforms, uniforms);
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nattribute float aGroup;\nvarying float vGroup;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvGroup = aGroup;');
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', '#include <common>\nuniform vec4 uWeights;\nuniform vec3 uTint;\nvarying float vGroup;\nfloat tgLit;')
      .replace('#include <color_fragment>', `#include <color_fragment>
        float g = floor(vGroup + 0.5);
        tgLit = g < 0.5 ? uWeights.x : g < 1.5 ? uWeights.y : g < 2.5 ? uWeights.z : uWeights.w;
        diffuseColor.rgb = mix(diffuseColor.rgb, uTint, tgLit * 0.92);`)
      .replace('#include <emissivemap_fragment>', '#include <emissivemap_fragment>\ntotalEmissiveRadiance += uTint * tgLit * 0.22;');
  };
  return m;
}

// A white texture whose alpha is alpha(u, v), both 0..1. Written as pixels
// rather than drawn as a canvas gradient: Safari dithers canvas gradients with
// noise in each colour, and at alphas this faint the upload magnifies that
// noise into bright coloured specks across the floor.
function alphaTexture(w, h, alpha) {
  const data = new Uint8Array(w * h * 4);
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
    const k = (j * w + i) * 4;
    data[k] = data[k + 1] = data[k + 2] = 255;
    data[k + 3] = Math.round(255 * clamp01(alpha((i + 0.5) / w, (j + 0.5) / h)));
  }
  const t = new THREE.DataTexture(data, w, h);
  t.magFilter = THREE.LinearFilter;
  t.minFilter = THREE.LinearMipmapLinearFilter;
  t.generateMipmaps = true;
  t.needsUpdate = true;
  return t;
}
// Linear between [position, value] stops, as a canvas gradient would be.
const ramp = stops => x => {
  for (let i = 1; i < stops.length; i++) {
    const [x0, a] = stops[i - 1], [x1, b] = stops[i];
    if (x <= x1) return a + (b - a) * clamp01((x - x0) / (x1 - x0));
  }
  return stops[stops.length - 1][1];
};

// ── The floor: a shadow, a pool of light, and the circle the part turns in ─
function buildFloor(y) {
  const floor = new THREE.Group();
  floor.position.y = y;

  // Where the part's shadow falls. Everything on the floor lies a fraction
  // of a millimetre above it, closer than a phone's depth buffer can tell
  // apart, so nothing on the floor writes depth: each layer is drawn in turn
  // by renderOrder instead, and only the part can hide them.
  const shadow = new THREE.Mesh(new THREE.PlaneGeometry(900, 900),
    new THREE.ShadowMaterial({ opacity: 0.55, depthWrite: false }));
  shadow.rotation.x = -Math.PI / 2;
  shadow.receiveShadow = true;
  floor.add(shadow);

  // A pool of light under the part, so it stands on something.
  const poolAlpha = ramp([[0, 0.16], [0.45, 0.05], [1, 0]]);
  const poolMap = alphaTexture(256, 256, (u, v) => poolAlpha(2 * Math.hypot(u - 0.5, v - 0.5)));
  const pool = new THREE.Mesh(new THREE.PlaneGeometry(260, 260), new THREE.MeshBasicMaterial({
    map: poolMap, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
  }));
  pool.rotation.x = -Math.PI / 2;
  pool.position.y = 0.02;
  pool.renderOrder = 1;
  floor.add(pool);

  const flat = (geo, mat, lift) => {
    const m = new THREE.Mesh(geo, mat);
    m.rotation.x = -Math.PI / 2; m.position.y = lift; m.renderOrder = 2; floor.add(m); return m;
  };

  // The circle the part turns in, kept faint.
  const R = 62;
  flat(new THREE.RingGeometry(R - 0.22, R + 0.22, 256), new THREE.MeshBasicMaterial({
    color: 0xffffff, transparent: true, opacity: 0.14, depthWrite: false,
  }), 0.05);

  return floor;
}

// ── Callouts: dimension labels pinned to the part ───────────────────────
function buildCallouts(svg, list) {
  const NS = 'http://www.w3.org/2000/svg';
  return list.map(c => {
    const g = document.createElementNS(NS, 'g');
    g.setAttribute('class', 'callout');
    const leader = document.createElementNS(NS, 'polyline');
    leader.setAttribute('fill', 'none');
    leader.setAttribute('stroke', 'currentColor');
    leader.style.stroke = 'var(--text-dim)';
    const ring = document.createElementNS(NS, 'circle');
    ring.setAttribute('class', 'ring'); ring.setAttribute('r', 6);
    const pin = document.createElementNS(NS, 'circle');
    pin.setAttribute('class', 'pin'); pin.setAttribute('r', 2.5);
    const text = document.createElementNS(NS, 'text');
    text.textContent = c.label;
    g.append(leader, ring, pin, text);
    svg.append(g);
    return {
      ...c, g, leader, ring, pin, text,
      local: new THREE.Vector3(...c.at),
      facing: new THREE.Vector3(...c.facing),
      shown: 0, visible: true,
    };
  });
}

// ── Start ───────────────────────────────────────────────────────────────
export async function start(stage, config) {
  const renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: 'high-performance' });
  if (!renderer.getContext()) throw new Error('no WebGL');
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.75));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.9;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  stage.append(renderer.domElement);
  // A phone short of GPU memory (a reload, before the old page has let go of
  // its own) can take the context away; the canvas would hold its last frame.
  // The still stands in until the browser gives the context back.
  renderer.domElement.addEventListener('webglcontextlost', () => stage.classList.add('lost'));
  renderer.domElement.addEventListener('webglcontextrestored', () => stage.classList.remove('lost'));

  const bg = token('--bg'), brand = token('--brand'), partColour = token('--part');
  const edgeColour = token('--edge'), valid = token('--valid');

  const [part, info] = await Promise.all([
    loadPart(config.model),
    fetch(config.modelInfo).then(r => r.json()),
  ]);

  const scene = new THREE.Scene();
  scene.background = bg;
  scene.fog = new THREE.Fog(bg, 300, 820);
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.32;

  // Light: a warm-white key from above, the brand from behind as a rim, a
  // little cool fill so the shadowed side still reads.
  const key = new THREE.DirectionalLight(0xffffff, 1.5);
  key.position.set(70, 150, 90);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, { left: -90, right: 90, top: 90, bottom: -90, near: 10, far: 400 });
  key.shadow.radius = 5;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.3;
  scene.add(key);
  const rim = new THREE.DirectionalLight(brand, 1.1);
  rim.position.set(-110, 50, -120);
  scene.add(rim);
  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x101012, 0.35));

  // The part: Z up in the file, Y up here.
  const uniforms = {
    uWeights: { value: new THREE.Vector4(0, 0, 0, 0) },   // w defaults to 1
    uTint: { value: brand.clone() },
  };
  const surface = new THREE.Mesh(part.surface, partMaterial(partColour, uniforms));
  surface.castShadow = surface.receiveShadow = true;
  const edges = new THREE.LineSegments(part.lines,
    new THREE.LineBasicMaterial({ color: edgeColour, transparent: true, opacity: 0.55 }));
  const model = new THREE.Group();
  model.rotation.x = -Math.PI / 2;
  model.add(surface, edges);
  const spinner = new THREE.Group();
  spinner.add(model);
  scene.add(spinner);

  const floorY = -info.size_mm[2] / 2 - 0.05;
  const floor = buildFloor(floorY);
  scene.add(floor);

  // The camera never comes within ~130 mm of anything, and a near plane
  // pulled in closer than it needs throws away depth precision.
  const camera = new THREE.PerspectiveCamera(26, 1, 40, 3000);
  const target = new THREE.Vector3(0, -3, 0);

  // Render at HDR with MSAA, bloom the bright things, tone map at the end.
  const rt = new THREE.WebGLRenderTarget(1, 1, { type: THREE.HalfFloatType, samples: 4 });
  const composer = new EffectComposer(renderer, rt);
  composer.addPass(new RenderPass(scene, camera));
  const bloom = new UnrealBloomPass(new THREE.Vector2(1, 1), config.bloom, 0.5, 2.2);
  if (config.bloom > 0) composer.addPass(bloom);
  composer.addPass(new OutputPass());

  const svg = document.getElementById('callouts');
  const callouts = config.callouts ? buildCallouts(svg, info.callouts) : [];

  // ── Page geometry, read on scroll and resize ─────────────────────────
  const hero = document.getElementById('home');
  const steps = [...document.querySelectorAll('.step')];
  const more = document.querySelector('.more');
  let W = 0, H = 0, narrow = false;
  function resize() {
    W = stage.clientWidth; H = stage.clientHeight;
    narrow = W < 760;
    renderer.setSize(W, H, false);
    composer.setSize(W, H);
    bloom.resolution.set(W / 2, H / 2);
    camera.aspect = W / H;
    svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  }
  addEventListener('resize', resize);
  resize();

  // ── Drag to turn ─────────────────────────────────────────────────────
  const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const baseSpin = reduced ? 0 : config.spin;
  let angle = -0.9, spin = reduced ? 0 : 1.6;   // arrives turning, settles to its pace
  let drag = null, tiltDrag = 0;
  hero.addEventListener('pointerdown', e => {
    if (e.target.closest('a, button, p, h1') || e.button !== 0) return;
    drag = { x: e.clientX, y: e.clientY, t: performance.now() };
    hero.classList.add('dragging');
    hero.setPointerCapture(e.pointerId);
  });
  hero.addEventListener('pointermove', e => {
    if (!drag) return;
    const now = performance.now(), dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    angle += dx * 0.009;
    tiltDrag = Math.max(-20, Math.min(30, tiltDrag + dy * 0.12));
    spin = dx * 0.009 / Math.max(0.001, (now - drag.t) / 1000);
    Object.assign(drag, { x: e.clientX, y: e.clientY, t: now });
  });
  const release = () => { drag = null; hero.classList.remove('dragging'); };
  hero.addEventListener('pointerup', release);
  hero.addEventListener('pointercancel', release);

  // A small lean towards the pointer, so the hero feels alive under it.
  let lean = 0, leanTarget = 0;
  addEventListener('pointermove', e => { leanTarget = (e.clientX / innerWidth - 0.5); }, { passive: true });

  // ── Where things are, per state ──────────────────────────────────────
  // Screen position of the part's centre (fractions of the stage), camera
  // distance, and the camera's pitch.
  const state = { sx: 0.68, sy: 0.5, dist: 280, pitch: 24, weights: new THREE.Vector4(0, 0, 0, 0), checked: 0 };
  // "all" is the checked step: the edges say it rather than the faces.
  const weightsFor = g => g === 'none' || g === 'all' || g == null ? new THREE.Vector4(0, 0, 0, 0)
    : new THREE.Vector4(...[0, 1, 2, 3].map(i => (i === +g ? 1 : 0)));

  let activeStep = -1, activeGroup = null, stageOpacity = 1, heroOut = 0;
  function readScroll() {
    heroOut = clamp01(scrollY / (hero.offsetHeight * 0.8));
    const mid = H * (narrow ? 0.62 : 0.5);
    activeStep = -1;
    steps.forEach((s, i) => {
      const r = s.getBoundingClientRect();
      if (r.top < mid && r.bottom > mid) activeStep = i;
    });
    steps.forEach((s, i) => s.classList.toggle('active', i === activeStep));
    activeGroup = activeStep >= 0 ? steps[activeStep].dataset.group : null;
    const mt = more.getBoundingClientRect().top;
    stageOpacity = smooth((mt - H * 0.3) / (H * 0.45));
    stage.style.opacity = stageOpacity;
    svg.style.opacity = stageOpacity;
  }
  addEventListener('scroll', readScroll, { passive: true });
  readScroll();

  // ── Occlusion, for the callouts ──────────────────────────────────────
  const ray = new THREE.Raycaster();
  const tmp = new THREE.Vector3(), tmpN = new THREE.Vector3(), centre = new THREE.Vector3();
  const nMat = new THREE.Matrix3();
  let frame = 0;

  // ── The loop ─────────────────────────────────────────────────────────
  let last = performance.now(), first = true;
  stage.classList.add('ready');
  document.body.classList.add('stage-live');   // the steps dim and light up from here
  renderer.setAnimationLoop(now => {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    if (stageOpacity <= 0.001 || document.hidden) return;

    // Targets for this moment of the page.
    const inFeatures = heroOut >= 1 && activeStep >= 0;
    const heroT = smooth(heroOut);
    const step = inFeatures ? steps[activeStep] : null;
    const t = {
      sx: narrow ? 0.5 : THREE.MathUtils.lerp(0.68, 0.71, heroT),
      sy: narrow ? THREE.MathUtils.lerp(0.27, 0.3, heroT) : 0.5,
      dist: THREE.MathUtils.lerp(270, 245, heroT),
      pitch: step ? +step.dataset.pitch : 24,
    };
    // Pull back until the part fits its share of the width: all of it on a
    // phone, under half of it beside the words.
    const across = 2 * Math.tan(13 * DEG) * camera.aspect;
    t.dist = Math.max(t.dist, narrow ? 118 / across : 88 / (0.44 * across));
    if (first) { Object.assign(state, { sx: t.sx, sy: t.sy, dist: t.dist, pitch: t.pitch }); first = false; }
    const rate = 3.2;
    state.sx = damp(state.sx, t.sx, rate, dt);
    state.sy = damp(state.sy, t.sy, rate, dt);
    state.dist = damp(state.dist, t.dist, rate, dt);
    state.pitch = damp(state.pitch, t.pitch + tiltDrag * (1 - heroT), 2.4, dt);
    if (!drag) tiltDrag = damp(tiltDrag, 0, 1.2, dt);
    const w = weightsFor(inFeatures ? activeGroup : null);
    for (const k of ['x', 'y', 'z', 'w']) state.weights[k] = damp(state.weights[k], w[k], 5, dt);
    state.checked = damp(state.checked, inFeatures && activeGroup === 'all' ? 1 : 0, 5, dt);
    edges.material.color.copy(edgeColour).lerp(valid, state.checked);
    edges.material.opacity = 0.55 + 0.45 * state.checked;
    uniforms.uWeights.value.copy(state.weights);

    // Turn.
    if (!drag) {
      spin = damp(spin, baseSpin, 0.9, dt);
      angle += spin * dt;
    }
    spinner.rotation.y = angle;
    lean = damp(lean, reduced ? 0 : leanTarget, 2, dt);

    // Camera.
    const yaw = (18 + lean * 6) * DEG, pitch = state.pitch * DEG;
    camera.position.set(
      target.x + state.dist * Math.cos(pitch) * Math.sin(yaw),
      target.y + state.dist * Math.sin(pitch),
      target.z + state.dist * Math.cos(pitch) * Math.cos(yaw));
    camera.lookAt(target);
    camera.setViewOffset(W, H, -(state.sx - 0.5) * W, -(state.sy - 0.5) * H, W, H);
    camera.updateProjectionMatrix();

    composer.render();
    placeCallouts(dt);
  });

  function placeCallouts(dt) {
    if (!callouts.length) return;
    model.updateWorldMatrix(true, true);
    nMat.getNormalMatrix(model.matrixWorld);
    centre.set(0, 0, 0).project(camera);
    const cx = (centre.x + 1) / 2 * W, cy = (1 - centre.y) / 2 * H;
    const checkOcclusion = frame++ % 4 === 0;
    const heroShow = 1 - smooth(heroOut * 2.2);

    for (const c of callouts) {
      tmp.copy(c.local).applyMatrix4(model.matrixWorld);
      tmpN.copy(c.facing).applyMatrix3(nMat).normalize();
      const toCam = camera.position.clone().sub(tmp).normalize();
      let visible = tmpN.dot(toCam) > 0.12;
      if (visible && checkOcclusion) {
        ray.set(tmp.clone().addScaledVector(toCam, 0.4), toCam);
        c.visible = ray.intersectObject(surface, false).length === 0;
      }
      visible = visible && c.visible;

      // Shown in the hero, and in the features only beside its own step.
      const inStep = heroOut >= 1 && activeGroup != null &&
        (activeGroup === 'none' || +activeGroup === c.group);
      const want = visible ? Math.max(heroShow, inStep ? 1 : 0) : 0;
      c.shown = damp(c.shown, want, 8, dt);
      c.g.style.opacity = c.shown.toFixed(3);
      if (c.shown < 0.01) continue;

      tmp.project(camera);
      const x = (tmp.x + 1) / 2 * W, y = (1 - tmp.y) / 2 * H;
      // Out from the part's centre, then a short shelf to the label.
      let dx = x - cx, dy = y - cy;
      const len = Math.hypot(dx, dy) || 1;
      dx /= len; dy /= len;
      const reach = narrow ? 34 : 58;
      const ex = x + dx * reach, ey = y + dy * reach * 0.8;
      // Pointing away from the part, unless that runs the label off the edge.
      const shelf = narrow ? 10 : 16, textW = c.label.length * 7.4 + 6;
      let side = dx >= 0 ? 1 : -1;
      if (side > 0 && ex + shelf + textW > W - 8) side = -1;
      else if (side < 0 && ex - shelf - textW < 8) side = 1;
      const sx = ex + side * shelf;
      c.leader.setAttribute('points', `${x},${y} ${ex},${ey} ${sx},${ey}`);
      c.pin.setAttribute('cx', x); c.pin.setAttribute('cy', y);
      c.ring.setAttribute('cx', x); c.ring.setAttribute('cy', y);
      c.text.setAttribute('x', sx + side * 6);
      c.text.setAttribute('y', ey + 4);
      c.text.setAttribute('text-anchor', side > 0 ? 'start' : 'end');
    }
  }
}
