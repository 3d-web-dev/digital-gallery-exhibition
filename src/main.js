import './style.css'
import $ from 'jquery';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'

import { Utils } from './components/utils.js';
import CameraController from './components/cameraController.js';
import KeyController from './components/keyController.js';
import MouseController from './components/mouseController.js'
import Building from './components/Building.js';
import Gallery from './components/Gallery.js';

export class App {

    constructor(container) {
        this.container = container;
        this.scene = Utils.BuildScene();
        this.camera = Utils.BuildCamera(this.container);

        this.renderer = Utils.BuildRenderer(this.container);

        // const orbit = new OrbitControls(this.camera, this.renderer.domElement);
        // this.camera.position.set(0, 50, 30);


        this.light = Utils.BuildLights(this.scene, this.camera);

        Utils.Resizer(container, this.camera, this.renderer);

    }

    async start() {

        // this._model = await Utils.loadModel(this.scene, this.renderer);


        const building = new Building(this.scene);

        const cameraController = new CameraController(this.camera, building, this.light);
        const keyController = new KeyController(cameraController);
        const mouseController = new MouseController(cameraController);


        const gallery = new Gallery(this.scene, building);
        gallery.build();



        $('#loader').hide();
        this.animate();
    }


    animate() {
        requestAnimationFrame(this.animate.bind(this));
        this.renderer.render(this.scene, this.camera);
    }
}


window.addEventListener('DOMContentLoaded', (event) => {
    console.log('DOM fully loaded and parsed');
    const container = $('#scene-container')[0];
    const myApp = new App(container);
    myApp.start();
});
