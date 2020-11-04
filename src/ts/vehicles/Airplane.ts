import * as THREE from 'three';
import * as CANNON from 'cannon';

import { Vehicle } from './Vehicle';
import { IControllable } from '../interfaces/IControllable';
import { IWorldEntity } from '../interfaces/IWorldEntity';
import { KeyBinding } from '../core/KeyBinding';
import { SpringSimulator } from '../physics/spring_simulation/SpringSimulator';
import * as Utils from '../core/FunctionLibrary';
import { EntityType } from '../enums/EntityType';

export class Airplane extends Vehicle implements IControllable, IWorldEntity
{
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

	constructor(gltf: any)
	{
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

	public noDirectionPressed(): boolean
	{
		let result = 
		!this.actions.throttle.isPressed &&
		!this.actions.brake.isPressed &&
		!this.actions.yawLeft.isPressed &&
		!this.actions.yawRight.isPressed &&
		!this.actions.rollLeft.isPressed &&
		!this.actions.rollRight.isPressed;

		return result;
	}

	public update(timeStep: number): void
	{
		super.update(timeStep);
		
		// Rotors visuals
		if (this.controllingCharacter !== undefined)
		{
			if (this.enginePower < 1) this.enginePower += timeStep * 0.4;
			if (this.enginePower > 1) this.enginePower = 1;
		}
		else
		{
			if (this.enginePower > 0) this.enginePower -= timeStep * 0.12;
			if (this.enginePower < 0) this.enginePower = 0;
		}
		this.rotor.rotateX(this.enginePower * timeStep * 60);

		// Steering
		if (this.rayCastVehicle.numWheelsOnGround > 0)
		{
			if ((this.actions.yawLeft.isPressed || this.actions.rollLeft.isPressed)
				&& !this.actions.yawRight.isPressed && !this.actions.rollRight.isPressed)
			{
				this.steeringSimulator.target = 0.8;
			}
			else if ((this.actions.yawRight.isPressed || this.actions.rollRight.isPressed)
				&& !this.actions.yawLeft.isPressed && !this.actions.rollLeft.isPressed)
			{
				this.steeringSimulator.target = -0.8;
			}
			else
			{
				this.steeringSimulator.target = 0;
			}
		}
		else
		{
			this.steeringSimulator.target = 0;
		}
		this.steeringSimulator.simulate(timeStep);
		this.setSteeringValue(this.steeringSimulator.position);

		const partsRotationAmount = 0.7;

		// Ailerons
		if (this.actions.rollLeft.isPressed && !this.actions.rollRight.isPressed)
		{
			this.aileronSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.rollLeft.isPressed && this.actions.rollRight.isPressed)
		{
			this.aileronSimulator.target = -partsRotationAmount;
		}
		else 
		{
			this.aileronSimulator.target = 0;
		}

		// Elevators
		if (this.actions.pitchUp.isPressed && !this.actions.pitchDown.isPressed)
		{
			this.elevatorSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.pitchUp.isPressed && this.actions.pitchDown.isPressed)
		{
			this.elevatorSimulator.target = -partsRotationAmount;
		}
		else
		{
			this.elevatorSimulator.target = 0;
		}

		// Rudder
		if (this.actions.yawLeft.isPressed && !this.actions.yawRight.isPressed)
		{
			this.rudderSimulator.target = partsRotationAmount;
		}
		else if (!this.actions.yawLeft.isPressed && this.actions.yawRight.isPressed)
		{
			this.rudderSimulator.target = -partsRotationAmount;
		}
		else 
		{
			this.rudderSimulator.target = 0;
		}

		// Run rotation simulators
		this.aileronSimulator.simulate(timeStep);
		this.elevatorSimulator.simulate(timeStep);
		this.rudderSimulator.simulate(timeStep);

		// Rotate parts
		this.leftAileron.rotation.y = this.aileronSimulator.position;
		this.rightAileron.rotation.y = -this.aileronSimulator.position;
		this.elevators.forEach((elevator) =>
		{
			elevator.rotation.y = this.elevatorSimulator.position;
		});
		this.rudder.rotation.y = this.rudderSimulator.position;
	}

	public physicsPreStep(body: CANNON.Body, plane: Airplane): void
	{
		let mid = new CANNON.Vec3(0, 0, 0);
		
		const velocity = body.quaternion.inverse().vmult(body.velocity);
		const currentSpeed = velocity.z;

		// Pitch
		if (plane.actions.pitchUp.isPressed)
		{
			body.applyLocalForce(new CANNON.Vec3(0, 5 * -currentSpeed, 0), new CANNON.Vec3(0, 0, -1));
		}
		if (plane.actions.pitchDown.isPressed)
		{
			body.applyLocalForce(new CANNON.Vec3(0, 5 * currentSpeed, 0), new CANNON.Vec3(0, 0, -1));
		}

		// Yaw
		if (plane.actions.yawLeft.isPressed)
		{
			body.applyLocalForce(new CANNON.Vec3(5 * -currentSpeed, 0, 0), new CANNON.Vec3(0, 0, -1));
		}
		if (plane.actions.yawRight.isPressed)
		{
			body.applyLocalForce(new CANNON.Vec3(5 * currentSpeed, 0, 0), new CANNON.Vec3(0, 0, -1));
		}

		// Roll
		if (plane.actions.rollLeft.isPressed)
		{
			body.applyLocalForce(new CANNON.Vec3(0, 5 * -currentSpeed, 0), new CANNON.Vec3(1, 0, 0));
			body.applyLocalForce(new CANNON.Vec3(0, 5 * currentSpeed, 0), new CANNON.Vec3(-1, 0, 0));
		}
		if (plane.actions.rollRight.isPressed)
		{
			body.applyLocalForce(new CANNON.Vec3(0, 5 * currentSpeed, 0), new CANNON.Vec3(1, 0, 0));
			body.applyLocalForce(new CANNON.Vec3(0, 5 * -currentSpeed, 0), new CANNON.Vec3(-1, 0, 0));
		}

		// Thrust
		let speedModifier = 0.03;
		if (plane.actions.throttle.isPressed && !plane.actions.brake.isPressed)
		{
			speedModifier = 0.12;
		}
		else
		{
			if (!plane.actions.throttle.isPressed && plane.actions.brake.isPressed) {
				speedModifier = 0;
			if (this.rayCastVehicle.numWheelsOnGround > 0)
				speedModifier -= 0.03;
		}

		// Drag
		let drag = new CANNON.Vec3(velocity.x * Math.abs(velocity.x) * -20,
					   velocity.y * Math.abs(velocity.y) * -100,
					   velocity.z * Math.abs(velocity.z) * -1);
		body.applyLocalForce(drag, new CANNON.Vec3(0, 0, -0.02));

		// Lift
		let lift = currentSpeed * Math.abs(currentSpeed) * 1.5;
		body.applyLocalForce(new CANNON.Vec3(0, lift, 0), mid);

		// Thrust
		body.applyLocalForce(new CANNON.Vec3(0, 0, 3000 * speedModifier * this.enginePower), new CANNON.Vec3(0, 0, 2));
	}

	public onInputChange(): void
	{
		super.onInputChange();

		const brakeForce = 100;

		if (this.actions.exitVehicle.justPressed && this.controllingCharacter !== undefined)
		{
			this.forceCharacterOut();
		}
		if (this.actions.wheelBrake.justPressed)
		{
			this.setBrake(brakeForce);
		}
		if (this.actions.wheelBrake.justReleased)
		{
			this.setBrake(0);
		}
		if (this.actions.view.justPressed)
		{
			this.toggleFirstPersonView();
		}
	}

	public readAirplaneData(gltf: any): void
	{
		gltf.scene.traverse((child) => {
			if (child.hasOwnProperty('userData'))
			{
				if (child.userData.hasOwnProperty('data'))
				{
					if (child.userData.data === 'rotor')
					{
						this.rotor = child;
					}
					if (child.userData.data === 'rudder')
					{
						this.rudder = child;
					}
					if (child.userData.data === 'elevator')
					{
						this.elevators.push(child);
					}
					if (child.userData.data === 'aileron')
					{
						if (child.userData.hasOwnProperty('side')) 
						{
							if (child.userData.side === 'left')
							{
								this.leftAileron = child;
							}
							else if (child.userData.side === 'right')
							{
								this.rightAileron = child;
							}
						}
					}
				}
			}
		});
	}

	public inputReceiverInit(): void
	{
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
