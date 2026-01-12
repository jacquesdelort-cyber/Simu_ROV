/** Scene setup utilities */

import * as THREE from 'three';

export function createScene() {
    const scene = new THREE.Scene();
    
    // Éclairage
    const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 5);
    scene.add(directionalLight);
    
    return scene;
}

