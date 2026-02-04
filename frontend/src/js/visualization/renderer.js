/** Three.js renderer setup */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class Visualization3D {
    constructor(container) {
        this.container = container;
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(
            75,
            container.clientWidth / container.clientHeight,
            0.1,
            1000
        );
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setClearColor(0x001122, 1);
        container.appendChild(this.renderer.domElement);
        
        // Contrôles caméra
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.camera.position.set(0, 10, 30);
        this.controls.update();
        
        // Objets 3D
        this.rovMesh = null;
        this.boatMesh = null;
        this.cableMesh = null;
        this.trajectoryLine = null;
        this.trajectoryPoints = [];
        
        // Mode caméra
        this.cameraMode = 'side'; // 'side', 'top', 'follow'
        
        this.createScene();
        this.animate();
        
        // Gérer redimensionnement
        window.addEventListener('resize', () => this.onWindowResize());
    }
    
    createScene() {
        // Éclairage
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        this.scene.add(directionalLight);
        
        // Grille de profondeur (plan horizontal)
        const gridHelper = new THREE.GridHelper(200, 50, 0x444444, 0x222222);
        gridHelper.rotation.x = Math.PI / 2;
        gridHelper.position.y = 0;
        this.scene.add(gridHelper);
        
        // Axes helper
        const axesHelper = new THREE.AxesHelper(10);
        this.scene.add(axesHelper);
        
        // ROV (boîte rouge)
        const rovGeometry = new THREE.BoxGeometry(0.8, 0.6, 0.5);
        const rovMaterial = new THREE.MeshPhongMaterial({ color: 0xff0000 });
        this.rovMesh = new THREE.Mesh(rovGeometry, rovMaterial);
        this.scene.add(this.rovMesh);
        
        // Bateau (boîte bleue)
        const boatGeometry = new THREE.BoxGeometry(5, 1, 2);
        const boatMaterial = new THREE.MeshPhongMaterial({ color: 0x0000ff });
        this.boatMesh = new THREE.Mesh(boatGeometry, boatMaterial);
        this.boatMesh.position.y = 0.5;
        this.scene.add(this.boatMesh);
        
        // Câble (sera mis à jour dynamiquement)
        this.cableGeometry = null;
        const cableMaterial = new THREE.MeshPhongMaterial({ color: 0x888888 });
        this.cableMesh = null;
        
        // Trajectoire ROV
        const trajectoryGeometry = new THREE.BufferGeometry();
        const trajectoryMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00 });
        this.trajectoryLine = new THREE.Line(trajectoryGeometry, trajectoryMaterial);
        this.scene.add(this.trajectoryLine);
    }
    
    updateState(state) {
        // Mettre à jour ROV
        this.rovMesh.position.set(state.rov.x, state.rov.y, 0);
        
        // Mettre à jour bateau
        this.boatMesh.position.set(state.boat.x, 0.5, 0);
        
        // Mettre à jour câble
        if (state.cable && state.cable.nodes && state.cable.nodes.length > 0) {
            const points = state.cable.nodes.map(node => 
                new THREE.Vector3(node.x, node.y, 0)
            );
            
            // Créer courbe lisse
            const curve = new THREE.CatmullRomCurve3(points);
            
            // Supprimer ancien câble
            if (this.cableMesh) {
                this.scene.remove(this.cableMesh);
            }
            
            // Créer nouveau câble (tube)
            const tubeGeometry = new THREE.TubeGeometry(curve, points.length * 2, 0.02, 8, false);
            const cableMaterial = new THREE.MeshPhongMaterial({ color: 0x888888 });
            this.cableMesh = new THREE.Mesh(tubeGeometry, cableMaterial);
            this.scene.add(this.cableMesh);
        }
        
        // Mettre à jour trajectoire
        this.trajectoryPoints.push(new THREE.Vector3(state.rov.x, state.rov.y, 0));
        if (this.trajectoryPoints.length > 1000) {
            this.trajectoryPoints.shift(); // Garder seulement les 1000 derniers points
        }
        
        if (this.trajectoryPoints.length > 1) {
            this.trajectoryLine.geometry.setFromPoints(this.trajectoryPoints);
        }
        
        // Mettre à jour caméra selon mode
        this.updateCamera();
    }
    
    updateCamera() {
        if (this.cameraMode === 'side') {
            this.camera.position.set(0, 10, 30);
            this.camera.lookAt(0, 5, 0);
        } else if (this.cameraMode === 'top') {
            this.camera.position.set(0, 50, 0);
            this.camera.lookAt(0, 0, 0);
        } else if (this.cameraMode === 'follow' && this.rovMesh) {
            const rovPos = this.rovMesh.position;
            this.camera.position.set(rovPos.x + 5, rovPos.y + 5, 10);
            this.camera.lookAt(rovPos);
        }
        this.controls.update();
    }
    
    setCameraMode(mode) {
        this.cameraMode = mode;
        this.updateCamera();
    }
    
    clearTrajectory() {
        this.trajectoryPoints = [];
        if (this.trajectoryLine) {
            this.trajectoryLine.geometry.setFromPoints([]);
        }
    }
    
    onWindowResize() {
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

