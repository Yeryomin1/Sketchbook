import * as THREE from 'three';
import * as CANNON from 'cannon';

import { Vehicle } from './Vehicle';
import { IControllable } from '../interfaces/IControllable';
import { IWorldEntity } from '../interfaces/IWorldEntity';
import { KeyBinding } from '../core/KeyBinding';
import { SpringSimulator } from '../physics/spring_simulation/SpringSimulator';
import * as Utils from '../core/FunctionLibrary';
import { EntityType } from '../enums/EntityType';

export class Airplane extends Vehicle implements IControllable, IWorldEntity {
	public entityType: EntityType = EntityType.Airplane;
	public rotor: THREE.Object3D;
	public leftAileron: THREE.Object3D;
	public rightAileron: THREE.Object3D;
	public elevators: THREE.Object3D[] = [];
	public rudder: THREE.Object3D;

	private steeringSimulator: SpringSimulator;
	private aileronSimulator: SpringSimulator;
	private elevatorSimulator: SpringSimulator;
	private rudderSimulator: SpringSimulator;

	private enginePower: number = 0;
	private lastDrag: number = 0;

	constructor(gltf: any) {
		super(gltf, {
			radius: 0.12,
			suspensionStiffness: 150,
			suspensionRestLength: 0.25,
			dampingRelaxation: 5,
			dampingCompression: 5,
			directionLocal: new CANNON.Vec3(0, -1, 0),
			axleLocal: new CANNON.Vec3(-1, 0, 0),
			chassisConnectionPointLocal: new CANNON.Vec3(),
		});


		this.collision.angularDamping = 0.8;
		this.readAirplaneData(gltf);

		this.collision.preStep = (body: CANNON.Body) => { this.physicsPreStep(body, this); };

		this.actions = {
			'throttle': new KeyBinding('ShiftLeft'),
			'brake': new KeyBinding('Space'),
			'wheelBrake': new KeyBinding('KeyB'),
			'pitchUp': new KeyBinding('KeyS'),
			'pitchDown': new KeyBinding('KeyW'),
			'yawLeft': new KeyBinding('KeyQ'),
			'yawRight': new KeyBinding('KeyE'),
			'rollLeft': new KeyBinding('KeyA'),
			'rollRight': new KeyBinding('KeyD'),
			'exitVehicle': new KeyBinding('KeyF'),
			'seat_switch': new KeyBinding('KeyX'),
			'view': new KeyBinding('KeyV'),
		};

		this.steeringSimulator = new SpringSimulator(60, 10, 0.6);
		this.aileronSimulator = new SpringSimulator(60, 5, 0.6);
		this.elevatorSimulator = new SpringSimulator(60, 7, 0.6);
		this.rudderSimulator = new SpringSimulator(60, 10, 0.6);
	}

	public noDirectionPressed(): boolean {
		let result =
			!this.actions.throttle.isPressed &&
			!this.actions.brake.isPressed &&
			!this.actions.yawLeft.isPressed &&
			!this.actions.yawRight.isPressed &&
			!this.actions.rollLeft.isPressed &&
			!this.actions.rollRight.isPressed;

		return result;
	}

	public update(timeStep: number): void {
		super.update(timeStep);

		// Rotors visuals
		if (this.controllingCharacter !== undefined) {
			if (this.enginePower < 1) this.enginePower += timeStep * 0.4;
			if (this.enginePower > 1) this.enginePower = 1;
		}
		else {
			if (this.enginePower > 0) this.enginePower -= timeStep * 0.12;
			if (this.enginePower < 0) this.enginePower = 0;
		}
		this.rotor.rotateX(this.enginePower * timeStep * 60);

		// Steering
		if (this.rayCastVehicle.numWheelsOnGround > 0) {
			if ((this.actions.yawLeft.isPressed || this.actions.rollLeft.isPressed)
				&& !this.actions.yawRight.isPressed && !this.actions.rollRight.isPressed) {
				this.steeringSimulator.target = 0.8;
			}
			else if ((this.actions.yawRight.isPressed || this.actions.rollRight.isPressed)
				&& !this.actions.yawLeft.isPressed && !this.actions.rollLeft.isPressed) {
				this.steeringSimulator.target = -0.8;
			}
			else {
				this.steeringSimulator.target = 0;
			}
		}
		else {
			this.steeringSimulator.target = 0;
		}
		this.steeringSimulator.simulate(timeStep);
		this.setSteeringValue(this.steeringSimulator.position);

		const partsRotationAmount = 0.7;

		// Ailerons
		if (this.actions.rollLeft.isPressed && !this.actions.rollRight.isPressed) {
			this.aileronSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.rollLeft.isPressed && this.actions.rollRight.isPressed) {
			this.aileronSimulator.target = -partsRotationAmount;
		}
		else {
			this.aileronSimulator.target = 0;
		}

		// Elevators
		if (this.actions.pitchUp.isPressed && !this.actions.pitchDown.isPressed) {
			this.elevatorSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.pitchUp.isPressed && this.actions.pitchDown.isPressed) {
			this.elevatorSimulator.target = -partsRotationAmount;
		}
		else {
			this.elevatorSimulator.target = 0;
		}

		// Rudder
		if (this.actions.yawLeft.isPressed && !this.actions.yawRight.isPressed) {
			this.rudderSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.yawLeft.isPressed && this.actions.yawRight.isPressed) {
			this.rudderSimulator.target = -partsRotationAmount;
		}
		else {
			this.rudderSimulator.target = 0;
		}

		// Run rotation simulators
		this.aileronSimulator.simulate(timeStep);
		this.elevatorSimulator.simulate(timeStep);
		this.rudderSimulator.simulate(timeStep);

		// Rotate parts
		this.leftAileron.rotation.y = this.aileronSimulator.position;
		this.rightAileron.rotation.y = -this.aileronSimulator.position;
		this.elevators.forEach((elevator) => {
			elevator.rotation.y = this.elevatorSimulator.position;
		});
		this.rudder.rotation.y = this.rudderSimulator.position;
	}

	public physicsPreStep(body: CANNON.Body, plane: Airplane): void {
		const alphaMax = 0.35;
		const wingS = 0.7;
		const mid = new CANNON.Vec3(0, 0, 0);

		/*
				local axis direction:
				Х - left
				У - up
				Z - forward
		*/
		//local speed
		const velocity = body.quaternion.inverse().vmult(body.velocity);
		const currentSpeed = velocity.length();
		const squareSpeed = currentSpeed * currentSpeed;
		const height = 0;//temporary
		const dynamicPressure = squareSpeed * 1.225 * Math.pow(Math.E, -0.000141 * height) * 0.5;
		const beta = velocity.length() > 0.5 ? Math.asin(velocity.x / velocity.length()) : 0;
		const alpha = velocity.length() > 0.5 ? -Math.asin(velocity.y / (velocity.length() * Math.cos(beta))) : 0;

		//aerodynamic coef
		let cL = 0;
		if (Math.abs(alpha) < alphaMax)
			cL = 4.3 * alpha;
		else if (alpha <= -alphaMax && alpha > -0.7)
			cL = -4.3 * alpha - 3.1;
		else if (alpha >= alphaMax && alpha < 0.7)
			cL = -4.3 * alpha + 3.1;
		const liftCoef = cL;
		const cL0 = 0.5;
		const dragCoef = Math.abs(alpha) < Math.PI * 0.5 ? 0.35 + alpha * alpha : 0.35 + (Math.abs(alpha) - Math.PI * 0.5) * (Math.abs(alpha) - Math.PI * 0.5);

		let flowDirection = new CANNON.Vec3(0, 0, 0);
		flowDirection.copy(velocity);
		flowDirection.normalize();
		flowDirection.negate(flowDirection);

		//aerodynamic center (focus)
		const aeroCenterPos = new CANNON.Vec3(0, 0, -0.1);
		//pressure center
		const pressCenterPos = new CANNON.Vec3(0, 0, this.rayCastVehicle.numWheelsOnGround > 0 ? 0.2 : 0.05);

		// Pitch
		const pitchF = alpha < alphaMax ? squareSpeed * 0.04 : squareSpeed * 0.02;
		if (plane.actions.pitchUp.isPressed) {
			body.applyLocalForce(new CANNON.Vec3(0, -pitchF, 0), new CANNON.Vec3(0, 0, -1));
		}
		if (plane.actions.pitchDown.isPressed) {
			body.applyLocalForce(new CANNON.Vec3(0, pitchF, 0), new CANNON.Vec3(0, 0, -1));
		}

		// Yaw
		const yawF = beta < alphaMax ? squareSpeed * 0.04 : squareSpeed * 0.02;
		if (plane.actions.yawLeft.isPressed) {
			body.applyLocalForce(new CANNON.Vec3(-yawF, 0, 0), new CANNON.Vec3(0, 0, -1));
		}
		if (plane.actions.yawRight.isPressed) {
			body.applyLocalForce(new CANNON.Vec3(yawF, 0, 0), new CANNON.Vec3(0, 0, -1));
		}

		// Roll
		if (plane.actions.rollLeft.isPressed) {
			body.applyLocalForce(new CANNON.Vec3(0, 3 * -currentSpeed, 0), new CANNON.Vec3(1, 0, 0));
			body.applyLocalForce(new CANNON.Vec3(0, 3 * currentSpeed, 0), new CANNON.Vec3(-1, 0, 0));
		}
		if (plane.actions.rollRight.isPressed) {
			body.applyLocalForce(new CANNON.Vec3(0, 3 * currentSpeed, 0), new CANNON.Vec3(1, 0, 0));
			body.applyLocalForce(new CANNON.Vec3(0, 3 * -currentSpeed, 0), new CANNON.Vec3(-1, 0, 0));
		}

		// Thrust
		let trust = 0;
		if (plane.actions.throttle.isPressed && !plane.actions.brake.isPressed) {
			trust = 400 * this.enginePower;
		}
		let trustVec = new CANNON.Vec3(0, 0, trust);
		body.applyLocalForce(trustVec, mid);

		// Drag
		let drag = dragCoef * dynamicPressure * wingS;
		let dragVec = flowDirection.scale(drag);
		body.applyLocalForce(dragVec, aeroCenterPos);

		// Lift
		let lift0 = cL0 * dynamicPressure * wingS;
		let lift0Vec = new CANNON.Vec3(0, lift0 * Math.cos(alpha), lift0 * Math.sin(alpha));
		body.applyLocalForce(lift0Vec, pressCenterPos);
		let lift = liftCoef * dynamicPressure * wingS;
		let liftVec = new CANNON.Vec3(0, lift * Math.cos(alpha), lift * Math.sin(alpha));
		body.applyLocalForce(liftVec, aeroCenterPos);

		//Side
		let side = beta * dynamicPressure;
		let sideVec = new CANNON.Vec3(-side, 0, 0);
		body.applyLocalForce(sideVec, aeroCenterPos);

	}

	public onInputChange(): void {
		super.onInputChange();

		const brakeForce = 100;

		if (this.actions.exitVehicle.justPressed && this.controllingCharacter !== undefined) {
			this.forceCharacterOut();
		}
		if (this.actions.wheelBrake.justPressed) {
			this.setBrake(brakeForce);
		}
		if (this.actions.wheelBrake.justReleased) {
			this.setBrake(0);
		}
		if (this.actions.view.justPressed) {
			this.toggleFirstPersonView();
		}
	}

	public readAirplaneData(gltf: any): void {
		gltf.scene.traverse((child) => {
			if (child.hasOwnProperty('userData')) {
				if (child.userData.hasOwnProperty('data')) {
					if (child.userData.data === 'rotor') {
						this.rotor = child;
					}
					if (child.userData.data === 'rudder') {
						this.rudder = child;
					}
					if (child.userData.data === 'elevator') {
						this.elevators.push(child);
					}
					if (child.userData.data === 'aileron') {
						if (child.userData.hasOwnProperty('side')) {
							if (child.userData.side === 'left') {
								this.leftAileron = child;
							}
							else if (child.userData.side === 'right') {
								this.rightAileron = child;
							}
						}
					}
				}
			}
		});
	}

	public inputReceiverInit(): void {
		super.inputReceiverInit();

		this.world.updateControls([
			{
				keys: ['Shift'],
				desc: 'Accelerate'
			},
			{
				keys: ['Space'],
				desc: 'Decelerate'
			},
			{
				keys: ['W', 'S'],
				desc: 'Elevators'
			},
			{
				keys: ['A', 'D'],
				desc: 'Ailerons'
			},
			{
				keys: ['Q', 'E'],
				desc: 'Rudder / Steering'
			},
			{
				keys: ['B'],
				desc: 'Brake'
			},
			{
				keys: ['V'],
				desc: 'View select'
			},
			{
				keys: ['F'],
				desc: 'Exit vehicle'
			},
			{
				keys: ['Shift', '+', 'R'],
				desc: 'Respawn'
			},
			{
				keys: ['Shift', '+', 'C'],
				desc: 'Free camera'
			},
		]);
	}
}