import * as THREE from 'three';
import { Reflector } from 'three/examples/jsm/objects/Reflector.js';

const thickness = 0.5;
const width = 28;
const depth = 10;
const height = 12;

export default class Building {


    constructor(scene) {
        this.scene = scene;

        this.floorWidth = 10 * width;

        this.width = width; //public
        this.height = height; //public
        this.depth = depth; //public

        this.walls = []; //public
        this.rooms = []; //public

        this.floor = new THREE.Mesh();
        this.ceiling = new THREE.Mesh();

        this.roomMat = this.getMaterial('room', 0.05, 0.05);
        this.wallMat = this.getMaterial('wall', 2, 2);
        this.ceilingMat = this.getMaterial('ceiling', 20, 1);
        this.floorMat = this.getMaterial('floor', 50, 5);
        this.floorMat.roughness = 0;

        this.build()
    }

    build() {

        this.buildFloor();
        this.buildCeiling();

        const wall = this.buildWall();
        const room = this.buildRoom();


        for (var i = 0; i < 5; ++i) {
            const clonedWall = wall.clone();
            clonedWall.position.x = wall.position.x + i * 2 * width;
            clonedWall.userData['range'] = { a: -width / 2 + 2 * width * i, b: width / 2 + 2 * width * i };
            clonedWall.castShadow = true;
            clonedWall.receiveShadow = true;

            const clonedRoom = room.clone();
            clonedRoom.position.x = room.position.x + i * 2 * width;
            clonedRoom.userData['range'] = { a: width / 2 + 2 * width * i, b: width * 3 / 2 + 2 * width * i };
            clonedRoom.castShadow = true;
            clonedRoom.receiveShadow = true;

            this.scene.add(clonedWall, clonedRoom);
            this.walls.push(clonedWall);
            this.rooms.push(clonedRoom);
        }


    }

    getMaterial(meshName, wRepeat, hRepeat) {

        const loader = new THREE.TextureLoader();

        const baseMap = loader.load(`images/${meshName}/base.jpg`);


        const normalMap = loader.load(`images/${meshName}/normal.jpg`);

        baseMap.wrapS = baseMap.wrapT = THREE.RepeatWrapping;
        normalMap.wrapS = normalMap.wrapT = THREE.RepeatWrapping;

        baseMap.repeat.set(wRepeat, hRepeat);
        normalMap.repeat.set(wRepeat, hRepeat);


        const mat = new THREE.MeshStandardMaterial({
            map: baseMap,
            normalMap: normalMap,
        });

        mat.normalScale = new THREE.Vector2(2, 2);

        return mat;
    }

    buildFloor() {
        // const plane = new THREE.Mesh(
        //     new THREE.PlaneGeometry(this.floorWidth, 2 * (depth + 2 * thickness)), this.floorMat);

        const plane = new Reflector(
            new THREE.PlaneGeometry(this.floorWidth, 2 * (depth + 2 * thickness)),
            {
                clipBias: 0.003,
                textureWidth: window.innerWidth * window.devicePixelRatio,
                textureHeight: window.innerHeight * window.devicePixelRatio,
                color: 0x000000,
            });
        plane.rotation.x = -Math.PI / 2;
        plane.position.x += this.floorWidth / 2 - width / 2;
        plane.receiveShadow = true;
        this.scene.add(plane);
        this.floor = plane;
    }

    buildCeiling() {
        // const ceilingGeo = new THREE.PlaneGeometry(this.floorWidth, depth + 2 * thickness);
        const ceilingGeo = new THREE.BoxGeometry(this.floorWidth, 0.1, depth + 2 * thickness);

        // ceilingGeo.rotateX(-Math.PI / 2);
        ceilingGeo.translate(0, height, -depth / 2);

        const ceilingMesh = new THREE.Mesh(ceilingGeo, this.ceilingMat);

        ceilingMesh.position.x += this.floorWidth / 2 - width / 2;
        ceilingMesh.castShadow = true;
        this.scene.add(ceilingMesh);
        this.ceiling = ceilingMesh;
    }

    buildWall() {
        const wallGeo = new THREE.BoxGeometry(width, thickness, height + 1/* offset */);
        wallGeo.rotateX(-Math.PI / 2);
        wallGeo.translate(0, height / 2, 0);
        const wall = new THREE.Mesh(wallGeo, this.wallMat);
        return wall;
    }

    buildRoom() {
        const shape = new THREE.Shape();
        shape.moveTo(-width / 2, 0);
        shape.lineTo(-width / 2, depth);
        shape.lineTo(width / 2, depth);
        shape.lineTo(width / 2, 0);
        shape.lineTo(width / 2 + thickness, 0);
        shape.lineTo(width / 2 + thickness, depth + thickness);
        shape.lineTo(-width / 2 - thickness, depth + thickness);
        shape.lineTo(-width / 2 - thickness, 0);
        const geo = new THREE.ExtrudeGeometry(shape, {
            steps: 1,
            depth: height + 1 /* offset */,
            bevelEnabled: false
        });
        geo.rotateX(-Math.PI / 2);
        geo.translate(0, 0, -thickness / 2);
        const room = new THREE.Mesh(geo, this.roomMat);
        room.position.x = width;
        room.position.y = -0.5;
        return room;
    }

}


