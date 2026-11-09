// =====================================================================
//  materials.js — display styles for the skeleton.
//   'xray' : additive view-angle (fresnel) shader. Every surface adds light,
//            depth test off → overlapping bones get brighter, like a
//            projection radiograph; silhouettes (tangential cortex) glow.
//            Cheap enough for mobile GPUs (no textures, no extra passes).
//   'bone' : lit ivory MeshStandardMaterial.
// =====================================================================
import * as THREE from 'three';

const XRAY_VERT = /* glsl */`
#include <common>
#include <skinning_pars_vertex>
varying vec3 vN;
varying vec3 vV;
void main() {
  #include <beginnormal_vertex>
  #include <skinbase_vertex>
  #include <skinnormal_vertex>
  #include <begin_vertex>
  #include <skinning_vertex>
  #include <project_vertex>
  vN = normalize(normalMatrix * objectNormal);
  vV = normalize(-mvPosition.xyz);
}`;

const XRAY_FRAG = /* glsl */`
uniform vec3 uColor;
uniform float uBase;
uniform float uEdge;
uniform float uPower;
uniform float uOpacity;
varying vec3 vN;
varying vec3 vV;
void main() {
  float c = abs(dot(normalize(vN), normalize(vV)));
  float rim = pow(1.0 - c, uPower);
  float a = (uBase + uEdge * rim) * uOpacity;
  gl_FragColor = vec4(uColor * a, 1.0);
}`;

export function makeXrayMaterial({ color = 0xbfd8ff, base = 0.05, edge = 0.55, power = 2.2, opacity = 1 } = {}) {
  return new THREE.ShaderMaterial({
    uniforms: {
      uColor: { value: new THREE.Color(color) },
      uBase: { value: base }, uEdge: { value: edge }, uPower: { value: power }, uOpacity: { value: opacity },
    },
    vertexShader: XRAY_VERT,
    fragmentShader: XRAY_FRAG,
    // pure additive on colour, destination alpha untouched: over a transparent canvas the
    // browser's premultiplied compositing then ADDS the glow to whatever is behind the
    // canvas (the camera video), instead of painting opaque dark pixels.
    blending: THREE.CustomBlending,
    blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor,
    blendEquationAlpha: THREE.AddEquation, blendSrcAlpha: THREE.ZeroFactor, blendDstAlpha: THREE.OneFactor,
    premultipliedAlpha: false,
    transparent: true,
    depthWrite: false,
    depthTest: false,
    side: THREE.DoubleSide,
  });
}

export function createMaterialSet() {
  const m = {
    xray: makeXrayMaterial(),
    xrayHL: makeXrayMaterial({ color: 0xffb347, base: 0.12, edge: 0.9 }),
    bone: new THREE.MeshStandardMaterial({ color: 0xe9dfc9, roughness: 0.62, metalness: 0.0 }),
    boneHL: new THREE.MeshStandardMaterial({ color: 0xffc070, roughness: 0.5, emissive: 0x6a3800 }),
    xrayFaded: makeXrayMaterial({ color: 0x8090a8, base: 0.02, edge: 0.22 }),
    boneFaded: new THREE.MeshStandardMaterial({ color: 0x9a9488, roughness: 0.7, transparent: true, opacity: 0.28, depthWrite: false }),
    skinXray: makeXrayMaterial({ color: 0x7fa8d8, base: 0.015, edge: 0.18, power: 3.0 }),
    skinBone: new THREE.MeshStandardMaterial({ color: 0xd9b8a0, roughness: 0.8, transparent: true, opacity: 0.25, depthWrite: false }),
  };
  return m;
}
