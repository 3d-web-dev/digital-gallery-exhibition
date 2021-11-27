import $ from 'jquery';
export default class MouseController {

    constructor(cameraController) {

        this._cameraController = cameraController;

        this._panDelta = null;
        this._currentX = null;

        this.mouseClicked = false;

        $(window).on('mousedown', this.onMouseDown.bind(this));
        $(window).on('mousemove', this.onMouseMove.bind(this));
        $(window).on('mouseup', this.onMouseUp.bind(this));

        $(window).on('touchstart', this.onTouchStart.bind(this));
        $(window).on('touchmove', this.onTouchMove.bind(this));
        $(window).on('touchend', this.onTouchEnd.bind(this));

        window.addEventListener('wheel', this.onMouseWheel.bind(this));
    }

    onMouseDown(e) {
        this.mouseClicked = true;
        $('#scene-container').css('cursor', 'grabbing')
        this._panDelta = e.clientX;
        this._currentX = this._cameraController.scrollX;
    }

    onMouseMove(e) {
        if (this._panDelta !== null && this._currentX !== null) {
            const distance = e.clientX - this._panDelta;
            this._cameraController.scrollX = this._currentX + distance / 35; //speed
            this._cameraController.startMoving();
        }
    }

    onMouseUp(e) {
        $('#scene-container').css('cursor', 'grab')
        this._panDelta = null;
    }


    onTouchStart(e) {
        this.mouseClicked = true;
        this._panDelta = e.touches[0].pageX;
        this._currentX = this._cameraController.scrollX;
    }

    onTouchMove(e) {
        if (this._panDelta !== null && this._currentX !== null) {
            const distance = e.touches[0].pageX - this._panDelta;
            this._cameraController.scrollX = this._currentX + distance / 25;
            this._cameraController.startMoving();
        }
    }

    onMouseWheel(e) {
        if (this.mouseClicked) {

            const speed = 4;
            this._currentX = this._cameraController.scrollX;
            this._cameraController.scrollX = this._currentX - e.deltaY / 125 * speed;
            this._cameraController.startMoving();
        }
    }

    onTouchEnd(e) { this._panDelta = null; }
}
