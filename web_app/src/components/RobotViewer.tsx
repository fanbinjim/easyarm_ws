import { useEffect, useRef, useState, type PointerEvent } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { api, apiAssetUrl, apiText } from "../api/client";
import type { JointTarget, Telemetry } from "../api/types";
import type { PoseValues } from "../ui/PoseEditor";

type ViewState = "loading" | "error" | "empty" | "ready";

type Props = {
  token: string;
  telemetry: Telemetry | null;
  jointTarget: JointTarget | null;
  moveLTarget: PoseValues | null;
};

interface URDFRobot extends THREE.Object3D {
  joints: Record<string, { setJointValue: (v: number) => void }>;
}

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type URDFLoaderCtor = new () => any;

type MeshDone = (mesh: THREE.Object3D | null, error?: Error | unknown) => void;

type MeshLoadCb = (
  path: string,
  manager: THREE.LoadingManager,
  material: THREE.Material,
  done: MeshDone,
) => void;

type ViewCubeState = {
  scene: THREE.Scene;
  camera: THREE.OrthographicCamera;
  renderer: THREE.WebGLRenderer;
  group: THREE.Group;
  cube: THREE.Mesh;
  hoverMarker: THREE.Mesh;
  raycaster: THREE.Raycaster;
  pointer: THREE.Vector2;
  dragging: boolean;
  moved: boolean;
  lastX: number;
  lastY: number;
};

type ViewCubeRegion = {
  type: "face" | "edge" | "vertex";
  direction: THREE.Vector3;
  markerPosition: THREE.Vector3;
  markerScale: THREE.Vector3;
  label: string;
};

const ROS_X = new THREE.Vector3(1, 0, 0);
const ROS_Y = new THREE.Vector3(0, 0, -1);
const ROS_Z = new THREE.Vector3(0, 1, 0);
const DEFAULT_TARGET = new THREE.Vector3(0, 0.35, 0);
const DEFAULT_CAMERA = new THREE.Vector3(1.6, 1.2, 1.6);

let _URDFLoaderModule: { default: URDFLoaderCtor } | null = null;

async function loadURDFLoader(): Promise<URDFLoaderCtor> {
  if (!_URDFLoaderModule) {
    _URDFLoaderModule = await import("urdf-loader");
  }
  return _URDFLoaderModule.default;
}

function resolveMeshUrl(meshPath: string, assetBase: string): string {
  if (/^https?:\/\//.test(meshPath) || meshPath.startsWith("/")) {
    return apiAssetUrl(meshPath);
  }
  return apiAssetUrl(`${assetBase}/${meshPath.replace(/^\.\//, "")}`);
}

function extensionOf(url: string): string {
  const withoutQuery = url.split(/[?#]/, 1)[0] ?? url;
  return withoutQuery.slice(withoutQuery.lastIndexOf(".")).toLowerCase();
}

function createLabel(text: string, color: string, position: THREE.Vector3): THREE.Sprite {
  const canvas = document.createElement("canvas");
  canvas.width = 96;
  canvas.height = 96;
  const ctx = canvas.getContext("2d");
  if (ctx) {
    ctx.fillStyle = "rgba(255, 255, 255, 0.86)";
    ctx.beginPath();
    ctx.arc(48, 48, 28, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = color;
    ctx.font = "700 44px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, 48, 50);
  }
  const texture = new THREE.CanvasTexture(canvas);
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
  const sprite = new THREE.Sprite(material);
  sprite.position.copy(position);
  sprite.scale.set(0.22, 0.22, 0.22);
  return sprite;
}

function createChamferedBoxGeometry(size: number, chamfer: number): THREE.BufferGeometry {
  const half = size / 2;
  const inner = half - chamfer;
  const positions: number[] = [];
  const groups: Array<{ start: number; count: number; materialIndex: number }> = [];

  const addTriangle = (a: THREE.Vector3, b: THREE.Vector3, c: THREE.Vector3, materialIndex: number) => {
    const start = positions.length / 3;
    positions.push(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
    groups.push({ start, count: 3, materialIndex });
  };

  const addQuad = (a: THREE.Vector3, b: THREE.Vector3, c: THREE.Vector3, d: THREE.Vector3, materialIndex: number) => {
    addTriangle(a, b, c, materialIndex);
    addTriangle(a, c, d, materialIndex);
  };

  const v = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z);

  addQuad(v(half, -inner, -inner), v(half, inner, -inner), v(half, inner, inner), v(half, -inner, inner), 0);
  addQuad(v(-half, inner, -inner), v(-half, -inner, -inner), v(-half, -inner, inner), v(-half, inner, inner), 1);
  addQuad(v(-inner, half, -inner), v(inner, half, -inner), v(inner, half, inner), v(-inner, half, inner), 2);
  addQuad(v(-inner, -half, inner), v(inner, -half, inner), v(inner, -half, -inner), v(-inner, -half, -inner), 3);
  addQuad(v(-inner, -inner, half), v(inner, -inner, half), v(inner, inner, half), v(-inner, inner, half), 4);
  addQuad(v(inner, -inner, -half), v(-inner, -inner, -half), v(-inner, inner, -half), v(inner, inner, -half), 5);

  const chamferMaterialIndex = 6;
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      addQuad(
        v(sx * half, sy * inner, -inner),
        v(sx * inner, sy * half, -inner),
        v(sx * inner, sy * half, inner),
        v(sx * half, sy * inner, inner),
        chamferMaterialIndex,
      );
    }
  }

  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      addQuad(
        v(sx * half, -inner, sz * inner),
        v(sx * inner, -inner, sz * half),
        v(sx * inner, inner, sz * half),
        v(sx * half, inner, sz * inner),
        chamferMaterialIndex,
      );
    }
  }

  for (const sy of [-1, 1]) {
    for (const sz of [-1, 1]) {
      addQuad(
        v(-inner, sy * half, sz * inner),
        v(-inner, sy * inner, sz * half),
        v(inner, sy * inner, sz * half),
        v(inner, sy * half, sz * inner),
        chamferMaterialIndex,
      );
    }
  }

  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      for (const sz of [-1, 1]) {
        addTriangle(
          v(sx * half, sy * inner, sz * inner),
          v(sx * inner, sy * half, sz * inner),
          v(sx * inner, sy * inner, sz * half),
          chamferMaterialIndex,
        );
      }
    }
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  for (const group of groups) {
    geometry.addGroup(group.start, group.count, group.materialIndex);
  }
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

function createRosAxes(scale: number): THREE.Group {
  const group = new THREE.Group();
  group.add(new THREE.ArrowHelper(ROS_X, new THREE.Vector3(0, 0, 0), scale, 0xd53030, scale * 0.16, scale * 0.08));
  group.add(new THREE.ArrowHelper(ROS_Y, new THREE.Vector3(0, 0, 0), scale, 0x1e9e56, scale * 0.16, scale * 0.08));
  group.add(new THREE.ArrowHelper(ROS_Z, new THREE.Vector3(0, 0, 0), scale, 0x2364d2, scale * 0.16, scale * 0.08));
  return group;
}

function directionToRosLabels(direction: THREE.Vector3): string[] {
  const labels: string[] = [];
  if (direction.x !== 0) labels.push(`${direction.x > 0 ? "+" : "-"}X`);
  if (direction.z !== 0) labels.push(`${direction.z < 0 ? "+" : "-"}Y`);
  if (direction.y !== 0) labels.push(`${direction.y > 0 ? "+" : "-"}Z`);
  return labels;
}

function classifyViewCubeRegion(hitPoint: THREE.Vector3): ViewCubeRegion {
  const half = 0.36;
  const hoverOutset = half * 0.1;
  const vertexThreshold = half * 0.6;
  const edgeThreshold = half * 0.28;
  const vertexSize = half * 0.4;
  const edgeLength = half * 1.2;
  const faceCenterSize = half * 1.2;
  const faceThickness = 0.026;
  const values = [hitPoint.x, hitPoint.y, hitPoint.z] as const;
  const abs = values.map((v) => Math.abs(v));
  const normalAxis = abs[0] >= abs[1] && abs[0] >= abs[2] ? 0 : abs[1] >= abs[2] ? 1 : 2;
  const sideAxes = [0, 1, 2].filter((axis) => axis !== normalAxis);
  const signs = [0, 0, 0];
  signs[normalAxis] = Math.sign(values[normalAxis]) || 1;

  const [sideA, sideB] = sideAxes;
  if (abs[sideA] >= vertexThreshold && abs[sideB] >= vertexThreshold) {
    signs[sideA] = Math.sign(values[sideA]) || 1;
    signs[sideB] = Math.sign(values[sideB]) || 1;
  } else if (abs[sideA] > edgeThreshold || abs[sideB] > edgeThreshold) {
    const edgeAxis = abs[sideA] >= abs[sideB] ? sideA : sideB;
    signs[edgeAxis] = Math.sign(values[edgeAxis]) || 1;
  }

  const activeAxes = signs.filter((v) => v !== 0).length;
  const type = activeAxes === 1 ? "face" : activeAxes === 2 ? "edge" : "vertex";
  const direction = new THREE.Vector3(signs[0], signs[1], signs[2]);

  const markerScale = new THREE.Vector3(
    signs[0] === 0 ? (type === "face" ? faceCenterSize : edgeLength) : type === "face" ? faceThickness : vertexSize,
    signs[1] === 0 ? (type === "face" ? faceCenterSize : edgeLength) : type === "face" ? faceThickness : vertexSize,
    signs[2] === 0 ? (type === "face" ? faceCenterSize : edgeLength) : type === "face" ? faceThickness : vertexSize,
  );
  const markerPosition = new THREE.Vector3(
    signs[0] === 0 ? 0 : signs[0] * (half + hoverOutset - markerScale.x / 2),
    signs[1] === 0 ? 0 : signs[1] * (half + hoverOutset - markerScale.y / 2),
    signs[2] === 0 ? 0 : signs[2] * (half + hoverOutset - markerScale.z / 2),
  );

  const prefix = type === "face" ? "正视" : type === "edge" ? "棱边视角" : "顶点视角";
  return {
    type,
    direction,
    markerPosition,
    markerScale,
    label: `${prefix} ${directionToRosLabels(direction).join(" ")}`,
  };
}

function pickViewCubeRegion(viewCube: ViewCubeState, element: HTMLElement, clientX: number, clientY: number): ViewCubeRegion | null {
  const rect = element.getBoundingClientRect();
  viewCube.pointer.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  viewCube.pointer.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  viewCube.raycaster.setFromCamera(viewCube.pointer, viewCube.camera);
  const hit = viewCube.raycaster.intersectObject(viewCube.cube, false)[0];
  if (!hit) return null;
  return classifyViewCubeRegion(viewCube.cube.worldToLocal(hit.point.clone()));
}

function updateViewCubeHover(viewCube: ViewCubeState, region: ViewCubeRegion | null): void {
  viewCube.hoverMarker.visible = Boolean(region);
  if (!region) return;
  viewCube.hoverMarker.position.copy(region.markerPosition);
  viewCube.hoverMarker.scale.copy(region.markerScale);
}

function rosPositionToThree(pose: Pick<PoseValues, "x" | "y" | "z">): THREE.Vector3 {
  return new THREE.Vector3(pose.x, pose.z, -pose.y);
}

function createMoveLTargetMarker(): THREE.Group {
  const group = new THREE.Group();
  group.visible = false;

  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(0.025, 20, 20),
    new THREE.MeshBasicMaterial({ color: 0xf59e0b }),
  );
  group.add(sphere);

  const ringMaterial = new THREE.LineBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.85 });
  const ringGeometry = new THREE.BufferGeometry().setFromPoints(
    Array.from({ length: 65 }, (_, i) => {
      const angle = (i / 64) * Math.PI * 2;
      return new THREE.Vector3(Math.cos(angle) * 0.07, 0, Math.sin(angle) * 0.07);
    }),
  );
  const ring = new THREE.LineLoop(ringGeometry, ringMaterial);
  group.add(ring);

  const crossMaterial = new THREE.LineBasicMaterial({ color: 0x111827, transparent: true, opacity: 0.7 });
  const crossGeometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-0.08, 0, 0), new THREE.Vector3(0.08, 0, 0),
    new THREE.Vector3(0, -0.08, 0), new THREE.Vector3(0, 0.08, 0),
    new THREE.Vector3(0, 0, -0.08), new THREE.Vector3(0, 0, 0.08),
  ]);
  group.add(new THREE.LineSegments(crossGeometry, crossMaterial));

  return group;
}

function createGhostMaterial(): THREE.MeshPhongMaterial {
  return new THREE.MeshPhongMaterial({
    color: 0xf59e0b,
    transparent: true,
    opacity: 0.3,
    depthWrite: false,
    side: THREE.DoubleSide,
  });
}

function applyGhostMaterial(object: THREE.Object3D): void {
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) return;
    mesh.material = createGhostMaterial();
    mesh.renderOrder = 5;
  });
}

function createMeshLoader(assetBase: string, ghost = false): MeshLoadCb {
  return (meshPath, manager, material, done) => {
    const fullPath = resolveMeshUrl(meshPath, assetBase);
    const meshMaterial = ghost ? createGhostMaterial() : material || new THREE.MeshPhongMaterial({ color: 0x888888 });
    const finishFallback = (error?: unknown) => {
      if (error) {
        console.warn("Robot mesh fallback:", meshPath, error);
      }
      const geometry = new THREE.SphereGeometry(0.015, 8, 8);
      const mesh = new THREE.Mesh(geometry, meshMaterial);
      if (ghost) mesh.renderOrder = 5;
      done(mesh);
    };

    if (extensionOf(fullPath) === ".stl") {
      const stlLoader = new STLLoader(manager);
      stlLoader.load(
        fullPath,
        (geometry) => {
          const mesh = new THREE.Mesh(geometry, meshMaterial);
          if (ghost) mesh.renderOrder = 5;
          done(mesh);
        },
        undefined,
        finishFallback,
      );
      return;
    }

    if (extensionOf(fullPath) === ".dae") {
      const colladaLoader = new ColladaLoader(manager);
      colladaLoader.load(
        fullPath,
        (dae) => {
          if (!dae?.scene) {
            finishFallback(new Error(`empty DAE scene: ${meshPath}`));
            return;
          }
          if (ghost) {
            applyGhostMaterial(dae.scene);
          }
          done(dae.scene);
        },
        undefined,
        finishFallback,
      );
      return;
    }

    finishFallback(new Error(`unsupported mesh type: ${meshPath}`));
  };
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

function setCameraDirection(
  camera: THREE.PerspectiveCamera,
  controls: OrbitControls,
  direction: THREE.Vector3,
): void {
  const target = controls.target;
  const radius = camera.position.distanceTo(target) || 2;
  const next = direction.clone().normalize().multiplyScalar(radius).add(target);
  camera.position.copy(next);
  camera.lookAt(target);
  controls.update();
}

function rotateCameraByDelta(
  camera: THREE.PerspectiveCamera,
  controls: OrbitControls,
  dx: number,
  dy: number,
): void {
  const target = controls.target;
  const offset = camera.position.clone().sub(target);
  const spherical = new THREE.Spherical().setFromVector3(offset);
  spherical.theta -= dx * 0.012;
  spherical.phi = clamp(spherical.phi - dy * 0.012, 0.08, Math.PI - 0.08);
  offset.setFromSpherical(spherical);
  camera.position.copy(target).add(offset);
  camera.lookAt(target);
  controls.update();
}

function createViewCube(container: HTMLDivElement): ViewCubeState {
  const size = 132;
  const scene = new THREE.Scene();
  const camera = new THREE.OrthographicCamera(-1.3, 1.3, 1.3, -1.3, 0.1, 10);
  camera.position.set(0, 0, 4);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(size, size);
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);

  const group = new THREE.Group();
  scene.add(group);

  const cubeMaterials = [
    new THREE.MeshBasicMaterial({ color: 0xffd6d6, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
    new THREE.MeshBasicMaterial({ color: 0xffeeee, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
    new THREE.MeshBasicMaterial({ color: 0xd9f7d9, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
    new THREE.MeshBasicMaterial({ color: 0xeffbef, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
    new THREE.MeshBasicMaterial({ color: 0xd9e8ff, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
    new THREE.MeshBasicMaterial({ color: 0xf0f6ff, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
    new THREE.MeshBasicMaterial({ color: 0xf3f6f4, transparent: true, opacity: 0.92, side: THREE.DoubleSide }),
  ];
  const cube = new THREE.Mesh(createChamferedBoxGeometry(0.72, 0.72 * 0.03), cubeMaterials);
  cube.name = "view-cube";
  group.add(cube);

  const hoverMarker = new THREE.Mesh(
    new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: false, opacity: 0.5 }),
  );
  hoverMarker.visible = false;
  group.add(hoverMarker);

  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(cube.geometry),
    new THREE.LineBasicMaterial({ color: 0xd8e0dc, transparent: true, opacity: 0.72, linewidth: 1 }),
  );
  group.add(edges);

  group.add(createRosAxes(1.05));
  group.add(createLabel("X", "#d53030", ROS_X.clone().multiplyScalar(1.25)));
  group.add(createLabel("Y", "#1e9e56", ROS_Y.clone().multiplyScalar(1.25)));
  group.add(createLabel("Z", "#2364d2", ROS_Z.clone().multiplyScalar(1.25)));

  return {
    scene,
    camera,
    renderer,
    group,
    cube,
    hoverMarker,
    raycaster: new THREE.Raycaster(),
    pointer: new THREE.Vector2(),
    dragging: false,
    moved: false,
    lastX: 0,
    lastY: 0,
  };
}

export function RobotViewer({ token, telemetry, jointTarget, moveLTarget }: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const viewCubeRef = useRef<HTMLDivElement>(null);
  const [viewState, setViewState] = useState<ViewState>("loading");
  const [errorMsg, setErrorMsg] = useState("");
  const [viewCubeHint, setViewCubeHint] = useState("");
  const [sceneVersion, setSceneVersion] = useState(0);
  const sceneRef = useRef<{
    robot: URDFRobot | null;
    ghostRobot: URDFRobot | null;
    joints: Map<string, { setJointValue: (v: number) => void }>;
    ghostJoints: Map<string, { setJointValue: (v: number) => void }>;
    jointNames: string[];
    camera: THREE.PerspectiveCamera | null;
    scene: THREE.Scene | null;
    renderer: THREE.WebGLRenderer | null;
    controls: OrbitControls | null;
    viewCube: ViewCubeState | null;
    moveLTargetMarker: THREE.Group | null;
    resizeObserver: ResizeObserver | null;
    abort: AbortController | null;
  }>({ robot: null, ghostRobot: null, joints: new Map(), ghostJoints: new Map(), jointNames: [], camera: null, scene: null, renderer: null, controls: null, viewCube: null, moveLTargetMarker: null, resizeObserver: null, abort: null });

  useEffect(() => {
    let disposed = false;
    const ctrl = new AbortController();
    sceneRef.current.abort = ctrl;
    const localScene = sceneRef.current;

    const init = async () => {
      try {
        const container = containerRef.current;
        const viewCubeContainer = viewCubeRef.current;
        if (!container || !viewCubeContainer || disposed) return;

        const width = container.clientWidth || 640;
        const height = Math.max(container.clientHeight || 400, 380);

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0xf5f7f6);

        const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 20);
        camera.position.copy(DEFAULT_CAMERA);
        camera.lookAt(DEFAULT_TARGET);

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(width, height);
        renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(renderer.domElement);
        renderer.domElement.addEventListener("contextmenu", (event) => event.preventDefault());

        const controls = new OrbitControls(camera, renderer.domElement);
        controls.target.copy(DEFAULT_TARGET);
        controls.enableDamping = true;
        controls.dampingFactor = 0.08;
        controls.enablePan = true;
        controls.enableZoom = true;
        controls.mouseButtons = {
          LEFT: THREE.MOUSE.ROTATE,
          MIDDLE: THREE.MOUSE.DOLLY,
          RIGHT: THREE.MOUSE.PAN,
        };
        controls.update();

        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(1, 2, 1);
        scene.add(dirLight);

        // Three.js 的 XZ 网格在这里映射为 ROS 的 XY 水平面（ROS Z 向上）。
        const gridHelper = new THREE.GridHelper(1.5, 20, 0xcccccc, 0xe0e0e0);
        scene.add(gridHelper);

        scene.add(createRosAxes(0.5));
        const moveLTargetMarker = createMoveLTargetMarker();
        scene.add(moveLTargetMarker);

        localScene.scene = scene;
        localScene.camera = camera;
        localScene.renderer = renderer;
        localScene.controls = controls;
        localScene.viewCube = createViewCube(viewCubeContainer);
        localScene.moveLTargetMarker = moveLTargetMarker;

        const resizeObserver = new ResizeObserver(() => {
          const nextWidth = container.clientWidth || 640;
          const nextHeight = Math.max(container.clientHeight || 400, 380);
          camera.aspect = nextWidth / nextHeight;
          camera.updateProjectionMatrix();
          renderer.setSize(nextWidth, nextHeight);
        });
        resizeObserver.observe(container);
        localScene.resizeObserver = resizeObserver;
        setViewState("ready");
        setSceneVersion((version) => version + 1);

        let model;
        let urdfXml = "";
        try {
          model = await api.robotModel;
          if (disposed) return;

          if (!model.joint_names || model.joint_names.length === 0) {
            console.warn("Robot model metadata has no joint_names; rendering scene without robot.");
            return;
          }

          urdfXml = await apiText(model.urdf_url);
        } catch (err) {
          console.warn("Robot model unavailable; rendering scene without robot.", err);
          return;
        }

        const assetBase = model.asset_base_url;
        try {
          const URDFLoaderCtor = await loadURDFLoader();
          const loader = new URDFLoaderCtor();
          const ghostLoader = new URDFLoaderCtor();

          (loader as unknown as { loadMeshCb: MeshLoadCb }).loadMeshCb = createMeshLoader(assetBase, false);
          (ghostLoader as unknown as { loadMeshCb: MeshLoadCb }).loadMeshCb = createMeshLoader(assetBase, true);

          (loader as unknown as { packages: (pkg: string) => string }).packages = (pkg: string) => `/api/robot/assets/${pkg}`;
          (ghostLoader as unknown as { packages: (pkg: string) => string }).packages = (pkg: string) => `/api/robot/assets/${pkg}`;

          const robot = (loader as unknown as { parse: (xml: string, workingPath?: string) => URDFRobot }).parse(urdfXml, "");
          robot.rotation.x = -Math.PI / 2;
          const ghostRobot = (ghostLoader as unknown as { parse: (xml: string, workingPath?: string) => URDFRobot }).parse(urdfXml, "");
          ghostRobot.rotation.x = -Math.PI / 2;
          ghostRobot.visible = false;
          applyGhostMaterial(ghostRobot);

          if (disposed) return;

          const jointMap = new Map<string, { setJointValue: (v: number) => void }>();
          const ghostJointMap = new Map<string, { setJointValue: (v: number) => void }>();
          if (robot && robot.joints) {
            for (const name of model.joint_names) {
              if (robot.joints[name]) {
                jointMap.set(name, robot.joints[name]);
              }
              if (ghostRobot.joints[name]) {
                ghostJointMap.set(name, ghostRobot.joints[name]);
              }
            }
          }

          scene.add(robot);
          scene.add(ghostRobot);
          localScene.robot = robot;
          localScene.ghostRobot = ghostRobot;
          localScene.joints = jointMap;
          localScene.ghostJoints = ghostJointMap;
          localScene.jointNames = model.joint_names;
        } catch (err) {
          console.warn("Robot URDF unavailable; rendering scene without robot.", err);
          return;
        }
      } catch (err) {
        if (disposed) return;
        const msg = err instanceof Error ? err.message : String(err);
        setErrorMsg(msg);
        setViewState("error");
      }
    };

    init();

    return () => {
      disposed = true;
      ctrl.abort();
      localScene.abort = null;
      localScene.resizeObserver?.disconnect();
      if (localScene.renderer?.domElement.parentElement) {
        localScene.renderer.domElement.parentElement.removeChild(localScene.renderer.domElement);
      }
      if (localScene.viewCube?.renderer.domElement.parentElement) {
        localScene.viewCube.renderer.domElement.parentElement.removeChild(localScene.viewCube.renderer.domElement);
      }
      localScene.controls?.dispose();
      localScene.renderer?.dispose();
      localScene.viewCube?.renderer.dispose();
      localScene.robot = null;
      localScene.ghostRobot = null;
      localScene.joints = new Map();
      localScene.ghostJoints = new Map();
      localScene.jointNames = [];
      localScene.camera = null;
      localScene.scene = null;
      localScene.renderer = null;
      localScene.controls = null;
      localScene.viewCube = null;
      localScene.moveLTargetMarker = null;
      localScene.resizeObserver = null;
    };
  }, [token]);

  useEffect(() => {
    const { ghostRobot, ghostJoints, jointNames } = sceneRef.current;
    if (!ghostRobot) return;
    if (!jointTarget) {
      ghostRobot.visible = false;
      return;
    }

    ghostRobot.visible = true;
    for (let i = 0; i < jointTarget.names.length; i++) {
      const joint = ghostJoints.get(jointTarget.names[i]);
      const value = jointTarget.positions[i];
      if (joint && Number.isFinite(value)) {
        joint.setJointValue(value);
      }
    }
  }, [jointTarget]);

  useEffect(() => {
    const marker = sceneRef.current.moveLTargetMarker;
    if (!marker) return;
    if (!moveLTarget) {
      marker.visible = false;
      return;
    }
    marker.position.copy(rosPositionToThree(moveLTarget));
    marker.visible = true;
  }, [moveLTarget]);

  useEffect(() => {
    const joints = telemetry?.latest_joints;
    if (!joints || viewState !== "ready") return;
    const jointMap = sceneRef.current.joints;
    for (let i = 0; i < joints.names.length; i++) {
      const name = joints.names[i];
      const j = jointMap.get(name);
      if (j) j.setJointValue(joints.positions[i]);
    }
  }, [telemetry, viewState]);

  useEffect(() => {
    if (viewState !== "ready") return;
    const { scene, camera, renderer, controls, viewCube } = sceneRef.current;
    if (!scene || !camera || !renderer || !controls) return;
    let animId = 0;
    const animate = () => {
      animId = requestAnimationFrame(animate);
      controls.update();
      if (viewCube) {
        viewCube.group.quaternion.copy(camera.quaternion).invert();
        viewCube.renderer.render(viewCube.scene, viewCube.camera);
      }
      renderer.render(scene, camera);
    };
    animate();
    return () => cancelAnimationFrame(animId);
  }, [viewState, sceneVersion]);

  const resetCamera = () => {
    const camera = sceneRef.current.camera;
    if (camera) {
      camera.position.set(1.5, 1.5, 1.5);
      const controls = sceneRef.current.controls;
      if (controls) {
        controls.target.copy(DEFAULT_TARGET);
        camera.lookAt(controls.target);
        controls.update();
      } else {
        camera.lookAt(DEFAULT_TARGET);
      }
    }
  };

  const handleCubePointerDown = (event: PointerEvent<HTMLDivElement>) => {
    const { viewCube } = sceneRef.current;
    if (!viewCube) return;
    viewCube.dragging = true;
    viewCube.moved = false;
    viewCube.lastX = event.clientX;
    viewCube.lastY = event.clientY;
    event.currentTarget.setPointerCapture(event.pointerId);
  };

  const handleCubePointerMove = (event: PointerEvent<HTMLDivElement>) => {
    const { camera, controls, viewCube } = sceneRef.current;
    if (!camera || !controls || !viewCube) return;

    if (!viewCube.dragging) {
      const region = pickViewCubeRegion(viewCube, event.currentTarget, event.clientX, event.clientY);
      updateViewCubeHover(viewCube, region);
      setViewCubeHint(region?.label ?? "");
      return;
    }

    const dx = event.clientX - viewCube.lastX;
    const dy = event.clientY - viewCube.lastY;
    if (Math.abs(dx) + Math.abs(dy) > 2) viewCube.moved = true;
    viewCube.lastX = event.clientX;
    viewCube.lastY = event.clientY;
    rotateCameraByDelta(camera, controls, dx, dy);
  };

  const handleCubePointerUp = (event: PointerEvent<HTMLDivElement>) => {
    const { camera, controls, viewCube } = sceneRef.current;
    if (!camera || !controls || !viewCube) return;
    viewCube.dragging = false;
    event.currentTarget.releasePointerCapture(event.pointerId);

    if (viewCube.moved) return;
    const region = pickViewCubeRegion(viewCube, event.currentTarget, event.clientX, event.clientY);
    if (!region) return;
    updateViewCubeHover(viewCube, region);
    setViewCubeHint(region.label);
    setCameraDirection(camera, controls, region.direction);
  };

  const handleCubePointerLeave = () => {
    const { viewCube } = sceneRef.current;
    if (!viewCube?.dragging) {
      if (viewCube) updateViewCubeHover(viewCube, null);
      setViewCubeHint("");
    }
  };

  return (
    <div className="robot-viewer">
      <div className="robot-viewer-canvas" ref={containerRef}>
        <div
          className="view-cube-container"
          ref={viewCubeRef}
          title="拖动旋转视角，点击面或棱切换视角"
          onPointerDown={handleCubePointerDown}
          onPointerMove={handleCubePointerMove}
          onPointerUp={handleCubePointerUp}
          onPointerLeave={handleCubePointerLeave}
        />
        {viewCubeHint && <div className="view-cube-hint">{viewCubeHint}</div>}
        {viewState === "loading" && (
          <div className="robot-viewer-overlay">
            <span>加载 3D 模型中...</span>
          </div>
        )}
        {viewState === "empty" && (
          <div className="robot-viewer-overlay">
            <span>3D 模型数据不可用</span>
          </div>
        )}
        {viewState === "error" && (
          <div className="robot-viewer-overlay robot-viewer-error">
            <span>3D 加载失败</span>
            <small>{errorMsg}</small>
          </div>
        )}
      </div>
      <div className="robot-viewer-toolbar">
        <button className="ghost-button" onClick={resetCamera}>Reset</button>
      </div>
    </div>
  );
}
