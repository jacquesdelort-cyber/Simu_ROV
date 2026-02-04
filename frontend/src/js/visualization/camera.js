/** Camera management */

import * as THREE from 'three';

export class CameraController {
    constructor(camera, controls) {
        this.camera = camera;
        this.controls = controls;
        this.mode = 'side';
    }
    
    setMode(mode) {
        this.mode = mode;
        this.update();
    }
    
    update() {
        if (this.mode === 'side') {
            this.camera.position.set(0, 10, 30);
            this.camera.lookAt(0, 5, 0);
        } else if (this.mode === 'top') {
            this.camera.position.set(0, 50, 0);
            this.camera.lookAt(0, 0, 0);
        }
        this.controls.update();
    }
    
    follow(target) {
        if (this.mode === 'follow' && target) {
            this.camera.position.set(
                target.x + 5,
                target.y + 5,
                10
            );
            this.camera.lookAt(target);
            this.controls.update();
        }
    }
}

