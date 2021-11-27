import $ from 'jquery';
export default class KeyController {

    constructor(cameraController) {

        this._cameraController = cameraController;
        this._direction = null;
        this._movingTimeout = -1;
        this._keySpeed = 2;
        this._FPS = 50;

        $(window).on('keydown', this._keyDown.bind(this));
        $(window).on('keyup', this._keyUp.bind(this));
        $(window).on('contextmenu', this._onContextMenu.bind(this));
    }

    _onContextMenu(event) {
        event.preventDefault();
    }


    _keyDown(e) {
        if (e.keyCode == 39) {

            this._direction = 'right';
        } else if (e.keyCode == 37) {

            this._direction = 'left';
        } else {

            this._direction = null
        }

        if (this._direction) {

            this.startChanging(); // update scrollX

            this._cameraController.startMoving();
        }
    }

    _keyUp(e) {

        this.stopChanging();
    }

    startChanging() {

        if (this._movingTimeout === -1) {
            this.loop();
        }
    }


    stopChanging() {
        clearTimeout(this._movingTimeout);
        this._movingTimeout = -1;
    }


    loop() {
        this._cameraController.scrollX += this._direction === 'left' ? this._keySpeed : -this._keySpeed;
        this._movingTimeout = window.setTimeout(this.loop.bind(this), 1000 / this._FPS, this._direction);
    }
}
