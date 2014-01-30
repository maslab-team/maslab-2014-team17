package robot;

import java.util.Iterator;

import robot.datautils.BoundedQueue;
import robot.datautils.MotionData;
import robot.datautils.SensorData;
import robot.datautils.SensorDataHistory;
import comm.MapleComm;
import devices.actuators.Cytron;
import devices.sensors.Encoder;

/**
 * Contains methods necessary to control the robot and to
 * retrieve sensor data.
 * 
 * Usage:
 * 	1) Create an instance of RobotController on the specified port:
 * 
 * 		RobotController rc = new RobotController("/dev/ttyACM0");
 * 
 * 	2) Call getMotionData() to update sensor data history in the robot
 * 	   controller and to access the recent data.
 * 	3) Call any combination of setXTarget functions.
 *  4) Call sendControl() to send the new instructions to the Maple.
 *  5) Repeat from step 2).
 * 
 * @author vmayar
 *
 */
public class RobotController {
	
	/** Set true to disable communication with the Maple. */
	private static final boolean NO_COMM = false;
	
	private static final boolean DEBUG = true;
	
	/** Wheel constants. */
	private static final double WHEEL_SEPARATION_IN_INCHES = 12.0;
	private static final double HALF_WHEEL_SEPARATION_IN_INCHES = WHEEL_SEPARATION_IN_INCHES / 2.0;
	private static final double WHEEL_DIAMETER_IN_INCHES = 3.88;
	private static final double WHEEL_RADIUS_IN_INCHES = WHEEL_DIAMETER_IN_INCHES / 2.0;
	
	/**
	 * Pins and ports.
	 */
	private static final int LEFT_CYTRON_PWM_PIN = 9;
	private static final int LEFT_CYTRON_DIR_PIN = 8;
	private static final int RIGHT_CYTRON_PWM_PIN = 6;
	private static final int RIGHT_CYTRON_DIR_PIN = 5;

	private static final int LEFT_ENCODER_PIN_A = 3;
	private static final int LEFT_ENCODER_PIN_B = 4;
	private static final int RIGHT_ENCODER_PIN_A = 2;
	private static final int RIGHT_ENCODER_PIN_B = 1;
	
	/** PID Controller paramaters. */
	private static final double P_ROT = 0.04;
	private static final double I_ROT = 0.000005;
	private static final double D_ROT = 0.0;
	private static final double P_TRANS = 0.0;
	private static final double I_TRANS = 0.00;
	private static final double D_TRANS = 0.0;
	private static final double MAX_SPEED = 0.20;
	
	private MapleComm comm;
	private Cytron leftWheel, rightWheel;
	private Encoder leftEncoder, rightEncoder;
	
	private SensorDataHistory sensorHistory;
	private MotionData motionData;
	private BoundedQueue<Error> errorHistory;
	
	/**
	 * Targets, in radians and inches.
	 */
	private double angleTarget;
	private double distanceTarget;
	
	/**
	 * Current error, in radians and inches.
	 */
	private Error error;
	
	/**
	 * Current position, in radians and inches.
	 */
	private double angleCurrent;
	// private double positionCurrent;
	
	RobotController(String port) {
		this.leftWheel = new Cytron(LEFT_CYTRON_DIR_PIN, LEFT_CYTRON_PWM_PIN);
		this.rightWheel = new Cytron(RIGHT_CYTRON_DIR_PIN, RIGHT_CYTRON_PWM_PIN);
		this.leftEncoder = new Encoder(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
		this.rightEncoder = new Encoder(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);
		this.sensorHistory = new SensorDataHistory();
		this.angleTarget = 0;
		this.distanceTarget = 0;
		this.angleCurrent = 0;
		updateError();
		this.errorHistory = new BoundedQueue<Error>();
		this.errorHistory.add(this.error);
		
		if(NO_COMM) {
			this.comm = null;
		} else {
			this.comm = new MapleComm(port);
			setUpShutdownHooks();
		}
	}
	
	/**
	 * Sets the current orientation of the robot.
	 * 
	 * @param angleInRadians
	 */
	void setCurrentAngle(double angleInRadians) {
		this.angleCurrent = toAngle(angleInRadians);
		updateError();
	}
	
	private void updateError() {
		double angleError = toAngle(this.angleTarget - this.angleCurrent);
		this.error = new Error(angleError, this.distanceTarget);
	}
	
	/*TODO:
	void setCurrentPosition(Point position) {
		
	}*/
	
	/**
	 * Stops motors on program exit.
	 */
	private void setUpShutdownHooks() {
		Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
			public void run() {
				if(!NO_COMM) {
					leftWheel.setSpeed(0);
					rightWheel.setSpeed(0);
					comm.transmit();
				}
			}
		}));
	}
	
	void setUpComm() {
		comm.registerDevice(leftWheel);
		comm.registerDevice(rightWheel);
		comm.registerDevice(rightEncoder);
		comm.registerDevice(leftEncoder);

		comm.initialize();
	}
	
	void setTarget(double angle, double distance) {
		this.angleTarget = toAngle(angle);
		this.distanceTarget = distance;
		updateError();
		errorHistory.clear();
		this.errorHistory.add(this.error);
	}
	
	void setRelativeTarget(double relAngle, double distance) {
		setTarget(this.angleCurrent + relAngle, distance);
	}
	
	private double toAngle(double angle) {
		double retAngle = angle % (2 * Math.PI);
		while(retAngle < -1.0 * Math.PI) {
			retAngle += 2 * Math.PI;
		} while(retAngle > Math.PI) {
			retAngle -= 2 * Math.PI;
		}
		return retAngle;
	}
	
	/**
	void setPositionTarget(Point target) {
		
	}
	
	void setAngleTarget(int target) {
		
	}
	
	void setVelocityTarget(Point target) {
		
	}
	
	void setAngularVelocityTarget(int target) {
		
	}*/
	
	/**
	 * Returns a useful summary of recent sensor data.  Adds the current
	 * sensor data to sensorHistory, stores a weighted average with
	 * acceleration in motionData, updates error fields, and updates
	 * current angle and position.
	 */
	MotionData getMotionData() {
		// Update data
		if(!NO_COMM) {
			comm.updateSensorData();
		}
		
		// Retrieve data
		SensorData data = new SensorData();
		data.leftWheelAngularSpeed = leftEncoder.getAngularSpeed();
		data.rightWheelAngularSpeed = rightEncoder.getAngularSpeed();
		data.leftWheelDeltaAngularDistance = leftEncoder.getDeltaAngularDistance();
		data.rightWheelDeltaAngularDistance = -1.0 * rightEncoder.getDeltaAngularDistance();
		data.time = System.currentTimeMillis();
		
		System.out.println("LEFT: " + data.leftWheelDeltaAngularDistance);
		System.out.println("RIGHT: " + data.rightWheelDeltaAngularDistance);
		
		// Add data to history
		sensorHistory.add(data);
		
		// Store new averages.
		motionData = sensorHistory.getMotionData();
		
		// Update error.
		//double encoderAngularSpeed = (data.rightWheelAngularSpeed - data.leftWheelAngularSpeed)
		//		* WHEEL_RADIUS_IN_INCHES / (2.0 * HALF_WHEEL_SEPARATION_IN_INCHES);
		double angleTraveled = (data.rightWheelDeltaAngularDistance - data.leftWheelDeltaAngularDistance)
				* WHEEL_RADIUS_IN_INCHES / (2.0 * HALF_WHEEL_SEPARATION_IN_INCHES);
		// Angle target is absolute.
		angleCurrent = toAngle(angleCurrent + angleTraveled);
		double distanceTraveled = (data.rightWheelDeltaAngularDistance + data.leftWheelDeltaAngularDistance)
				/ 2.0;
		// Distance target is relative.
		distanceTarget = distanceTarget - distanceTraveled;
		
		updateError();
		errorHistory.add(error);
		
		debug();
		
		// Return copy to prevent unexpected modification.
		return new MotionData(motionData);
	}
	
	
	/**
	 * Submits the control instructions to the robot.
	 */
	void sendControl() {
		int weight = 0, weightSum = 0;
		Error front, back;
		
		double angleErrorDerivative = 0, angleErrorIntegral = 0;
		double distanceErrorDerivative = 0, distanceErrorIntegral = 0;
		double angleError = error.angleError, distanceError = error.distanceError;
		// Calculate controller terms.
		Iterator<Error> itrFront = errorHistory.iterator();
		Iterator<Error> itrBack = errorHistory.iterator();
		for(int ctr = 0; ctr < 10; ++ctr) {
			if(!itrFront.hasNext()) {
				break;
			}
			front = itrFront.next();
		}
		while(itrFront.hasNext()) {
			front = itrFront.next();
			back = itrBack.next();
			long deltaTime = front.time - back.time;
			
			if(0 == deltaTime) {
				System.out.printf("Delta time is zero.");
				continue;
			}
			angleErrorDerivative += weight * (front.angleError - back.angleError)
					/ deltaTime;
			distanceErrorDerivative += weight * (front.distanceError - back.distanceError)
					/ deltaTime;
			
			angleErrorIntegral += front.angleError * deltaTime;
			distanceErrorIntegral += front.distanceError * deltaTime;

			weightSum += weight;
			++weight;
		}
		
		if(0 != weightSum) {
			angleErrorDerivative /= weightSum;
			distanceErrorDerivative /= weightSum;
		} /* else {
			No terms added, so derivatives are 0.
		} */
		
		// Angular controller:
		double rotationalControl = P_ROT * angleError
				+ I_ROT * angleErrorIntegral
				+ D_ROT * angleErrorDerivative;
		
		// Translational controller (depends on angle error):
		double translationalControl = (P_TRANS * distanceError
				+ I_TRANS * distanceErrorIntegral
				+ D_TRANS * distanceErrorDerivative)
				* 1.0 / (1.0 + 2 * Math.abs(angleError));

		double leftWheelControl = translationalControl - rotationalControl * HALF_WHEEL_SEPARATION_IN_INCHES;
		double rightWheelControl = translationalControl + rotationalControl * HALF_WHEEL_SEPARATION_IN_INCHES;
		
		// Cap control magnitude at MAX_SPEED.
		leftWheelControl = (Math.abs(leftWheelControl) > MAX_SPEED) ? Math.signum(leftWheelControl)*MAX_SPEED : leftWheelControl;
		rightWheelControl = (Math.abs(rightWheelControl) > MAX_SPEED) ? Math.signum(rightWheelControl)*MAX_SPEED : rightWheelControl;

		leftWheel.setSpeed(leftWheelControl);
		rightWheel.setSpeed(rightWheelControl);

		if(DEBUG) {
			System.out.printf("Rotational control:\n");
			System.out.printf("\tP_ROT:\t%.5f\n", P_ROT);
			System.out.printf("\tI_ROT:\t%.5f\n", I_ROT);
			System.out.printf("\tD_ROT:\t%.5f\n", D_ROT);
			System.out.printf("\tangleError:\t%.5f\n", angleError);
			System.out.printf("\tangleErrorIntegral:\t%.5f\n", angleErrorIntegral);
			System.out.printf("\tangleErrorDerivative:\t%.5f\n", angleErrorDerivative);
			System.out.printf("\trotational control:\t%.5f\n", rotationalControl);
			System.out.printf("Translational control:\n");
			System.out.printf("\tP_ROT:\t%.5f\n", P_TRANS);
			System.out.printf("\tI_ROT:\t%.5f\n", I_TRANS);
			System.out.printf("\tD_ROT:\t%.5f\n", D_TRANS);
			System.out.printf("\tangleError:\t%.5f\n", angleError);
			System.out.printf("\tdistanceErrorIntegral:\t%.5f\n", distanceErrorIntegral);
			System.out.printf("\tdistanceErrorDerivative:\t%.5f\n", distanceErrorDerivative);
			System.out.printf("\ttranslational control:\t%.5f\n", translationalControl);
			System.out.printf("Left wheel control:\t%.5f\n", leftWheelControl);
			System.out.printf("Right wheel control:\t%.5f\n", rightWheelControl);

			debug();
		}
		
		if(!NO_COMM) {
			comm.transmit();
		}
	}
	
	private void debug() {
		if(DEBUG) {
			System.out.printf("Controller info:\n");
			System.out.printf("\tangleCurrent:\t%.5f\n", angleCurrent);
			System.out.printf("\tangleError:\t%.5f\n", error.angleError);
			System.out.printf("\tangleTarget:\t%.5f\n", angleTarget);
			System.out.printf("\tdistanceError:\t%.5f\n", error.distanceError);
			System.out.printf("\tdistanceTarget:\t%.5f\n", distanceTarget);
		}
	}
	
	private static class Error {
		double angleError, distanceError;
		long time;
		
		Error(double angleError, double distanceError) {
			this.angleError = angleError;
			this.distanceError = distanceError;
			this.time = System.currentTimeMillis();
		}
	}
}
