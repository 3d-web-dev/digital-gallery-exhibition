import $ from 'jquery';
import * as THREE from 'three';
import Bodies from './bodies.js';

export default class Gallery {

    constructor(_scene, _building) {
        this.scene = _scene;
        this.building = _building;
    }

    build() {

        this.firstWall();

        let videoStarted = false;

        var video = $('<video />', {
            id: 'video',
            src: 'video/oil.mp4',
            type: 'video/mp4',
            autoplay: true,
            controls: false
        });
        video.css('display', 'none');
        video.attr('loop', 'loop');
        video.appendTo(document.body);


        ['keydown', 'mousedown', 'touchstart'].forEach(eventName => {
            $(window).on(eventName, e => {
                if (!videoStarted) {
                    videoStarted = true;
                    this.firstRoom()
                    this.secondWall()
                    this.secondRoom();
                    this.thirdWall();

                }
            })
        });
    }

    setPos(plane, which, nTH) {
        const targetBuilding = (which === 'room') ?
            this.building.rooms[nTH - 1] : this.building.walls[nTH - 1];

        plane.position.x = targetBuilding.position.x;
        plane.position.y = this.building.height / 2;
        plane.position.z = which.includes('room') ? -this.building.depth : 0 + .3;
        targetBuilding.add(plane);
        const inversionMatrix = new THREE.Matrix4();
        inversionMatrix.copy(targetBuilding.matrix).invert();
        plane.applyMatrix4(inversionMatrix);
    }

    firstWall() {
        const plane = new THREE.Mesh(
            new THREE.PlaneGeometry(this.building.width - 6, this.building.height - 3),
            new THREE.MeshStandardMaterial({
                map: new THREE.TextureLoader().load('images/wall1.png'),
                transparent: true
            })
        );

        this.setPos(plane, 'wall', 1);
    }

    firstRoom() {
        const video = document.getElementById('video');
        video.play();
        const texture = new THREE.VideoTexture(video);
        const plane = new THREE.Mesh(
            new THREE.PlaneGeometry(this.building.width - 4, this.building.height - 4),
            new THREE.MeshLambertMaterial({
                map: texture,
            })
        );
        this.setPos(plane, 'room', 1);
    }

    secondWall() {
        const plane = new THREE.Mesh(
            new THREE.PlaneGeometry(this.building.width - 8, this.building.height - 4),
            new THREE.MeshStandardMaterial({
                map: new THREE.TextureLoader().load('images/wall2.png'),
                transparent: true
            })
        );
        this.setPos(plane, 'wall', 2);
    }

    secondRoom() {
        const canvas = document.createElement('canvas');
        canvas.width = 1000
        canvas.height = 400
        document.body.appendChild(canvas);
        canvas.style.display = "none";
        canvas.style.zIndex = 0;

        const bodies = new Bodies(canvas)

        const texture = new THREE.CanvasTexture(canvas);

        const plane = new THREE.Mesh(
            new THREE.PlaneGeometry(this.building.width - 3, this.building.height - 4),
            new THREE.MeshStandardMaterial({
                map: texture
            })
        );

        bodies.run();

        var updateCanvas = () => {
            texture.needsUpdate = true;
            requestAnimationFrame(updateCanvas)
        }
        updateCanvas()


        this.setPos(plane, 'room', 2);
    }

    thirdWall() {
        const plane = new THREE.Mesh(
            new THREE.PlaneGeometry(this.building.width, this.building.height),
            new THREE.MeshStandardMaterial({
                map: new THREE.TextureLoader().load('images/wall3.png'),
                transparent: true
            })
        );
        this.setPos(plane, 'wall', 3);
    }

}