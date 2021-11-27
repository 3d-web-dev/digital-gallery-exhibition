import * as THREE from "three";
import { RGBELoader } from "three/examples/jsm/loaders/RGBELoader.js";
import { RectAreaLightUniformsLib } from "three/examples/jsm/lights/RectAreaLightUniformsLib.js";
const Utils = {
  BuildScene: function () {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x112288);
    const axis = new THREE.AxesHelper(3);
    axis.position.y = 0.1;
    // scene.add(axis);
    return scene;
  },

  BuildCamera: function (container) {
    const fov = 45;
    const near = 0.1;
    const far = 1000;
    const aspect = container.clientWidth / container.clientHeight;
    const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
    camera.position.set(0, 5.5, 16.2);
    camera.lookAt(0, 5.5, 0);
    return camera;
  },

  BuildLights: function (scene, camera) {
    RectAreaLightUniformsLib.init();

    const ambientLight = new THREE.HemisphereLight(0xffffff, 0xeeeeee, 0.2);

    scene.add(ambientLight);

    const spotLight = new THREE.SpotLight(0xffffff, 0.4);
    spotLight.angle = Math.PI / 2;
    spotLight.penumbra = 0.3;
    spotLight.position.set(8, 5, 5);
    spotLight.castShadow = true;
    spotLight.shadow.camera.near = 8;
    spotLight.shadow.camera.far = 200;
    spotLight.shadow.mapSize.width = 256;
    spotLight.shadow.mapSize.height = 256;
    spotLight.shadow.bias = -0.002;
    spotLight.shadow.radius = 3;
    const lightGroup = new THREE.Group();
    lightGroup.add(spotLight);
    scene.add(lightGroup);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
    dirLight.name = "Dir. Light";
    dirLight.position.set(0, 0, 10);
    dirLight.castShadow = true;
    dirLight.shadow.camera.near = 0.1;
    dirLight.shadow.camera.far = 500;
    dirLight.shadow.camera.right = 17;
    dirLight.shadow.camera.left = -17;
    dirLight.shadow.camera.top = 17;
    dirLight.shadow.camera.bottom = -17;
    dirLight.shadow.mapSize.width = 512;
    dirLight.shadow.mapSize.height = 512;
    dirLight.shadow.radius = 4;
    dirLight.shadow.bias = -0.0005;

    scene.add(dirLight);

    return lightGroup;
  },

  BuildControls: function (camera, renderer) {
    const orbit = new OrbitControls(camera, renderer.domElement, document);
    orbit.target.set(0, 0, 0);
    orbit.enableDamping = true;
    // orbit.enablePan = false;
    return orbit;
  },

  BuildRenderer: function (container) {
    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      preserveDrawingBuffer: true,
    });

    renderer.outputEncoding = THREE.sRGBEncoding;

    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.VSMShadowMap;
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.append(renderer.domElement);

    return renderer;
  },

  Resizer: function (container, camera, renderer) {
    const setSize = () => {
      camera.aspect = container.clientWidth / container.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(container.clientWidth, container.clientHeight);
      renderer.setPixelRatio(window.devicePixelRatio);
    };

    setSize();
    window.addEventListener("resize", setSize);
  },

  loadModel: async function (scene, renderer) {
    const [envTexture] = await Promise.all([
      new Promise((resolve) => {
        new RGBELoader()
          .setDataType(THREE.UnsignedByteType)
          .load("env/venice_sunset_1k.hdr", resolve);
      }),
    ]);

    const pmremGenerator = new THREE.PMREMGenerator(renderer);
    pmremGenerator.compileEquirectangularShader();
    const envMap = pmremGenerator.fromEquirectangular(envTexture).texture;
    scene.environment = envMap;
    scene.background = envMap;
    envTexture.dispose();
    pmremGenerator.dispose();

    // const model = modelData.scene;
    // model.traverse(m => {
    //     if (m.isMesh) {
    //         m.castShadow = true;
    //     }
    // })
    // model.position.y = 1;
    // scene.add(model);

    // return model;
  },
};

export { Utils };
