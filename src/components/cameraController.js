import gsap from 'gsap';

export default class CameraController {

    constructor(camera, buildings, light) {

        this.camera = camera;
        this.buildings = buildings;
        this.light = light;

        this._posZ = camera.position.z;
        this.scroller = {
            target: this.camera.position,
            ease: 0.1, // <= scroll speed
            x: 0,
            resizeRequest: 1,
            scrollRequest: 0,
        };

        this.scrollX = 0;

        this._requestId = null;

        this.updateBuildings()

    }

    startMoving() {
        this.scroller.scrollRequest++;
        if (!this._requestId) {
            this._requestId = requestAnimationFrame(this.updateScroller.bind(this));
        }
    }

    updateScroller() {
        this.scroller.x += (this.scrollX - this.scroller.x) * this.scroller.ease;
        if (Math.abs(this.scrollX - this.scroller.x) < 0.05) {
            this.scroller.x = this.scrollX;
            this.scroller.scrollRequest = 0;
        }

        gsap.to(this.scroller.target, {
            x: -this.scroller.x
            ,
            onUpdate: this.updateBuildings.bind(this)
        });
        this._requestId = this.scroller.scrollRequest > 0 ? requestAnimationFrame(this.updateScroller.bind(this)) : null;
    }

    updateBuildings() {
        const w = this.buildings.width;

        for (let i = 0; i < 5; ++i) {

            /* Camera In Wall */
            if (this.camera.position.x > this.buildings.walls[i].userData.range.a &&
                this.camera.position.x < this.buildings.walls[i].userData.range.b) {

                const rightRoom = this.buildings.rooms[i];
                const leftRoom = this.buildings.rooms[i === 0 ? 4 : i - 1];

                rightRoom.userData.range = {
                    a: rightRoom.position.x - w / 2, b: rightRoom.position.x + w / 2
                }
                leftRoom.userData.range = {
                    a: leftRoom.position.x - w / 2, b: leftRoom.position.x + w / 2
                }

                rightRoom.position.x = this.buildings.walls[i].position.x + w;
                leftRoom.position.x = this.buildings.walls[i].position.x - w;

                this.buildings.floor.position.x = this.buildings.ceiling.position.x = this.buildings.rooms[i].position.x;
            }

            /* Camera In Room  */
            if (this.camera.position.x > this.buildings.rooms[i].userData.range.a &&
                this.camera.position.x < this.buildings.rooms[i].userData.range.b) {

                const rightWall = this.buildings.walls[i === 4 ? 0 : i + 1];
                const leftWall = this.buildings.walls[i];

                rightWall.userData.range = {
                    a: rightWall.position.x - w / 2, b: rightWall.position.x + w / 2
                }
                leftWall.userData.range = {
                    a: leftWall.position.x - w / 2, b: leftWall.position.x + w / 2
                }

                rightWall.position.x = this.buildings.rooms[i].position.x + w;
                leftWall.position.x = this.buildings.rooms[i].position.x - w;

                this.buildings.floor.position.x = this.buildings.ceiling.position.x = this.buildings.rooms[i].position.x;
            }

            this.light.position.x = this.camera.position.x;

        }

    }
}

