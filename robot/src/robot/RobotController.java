package robot;

import java.util.Iterator;

import robot.datautils.BoundedQueue;
import robot.datautils.SensorData;

import comm.MapleComm;

import devices.actuators.Cytron;
import devices.sensors.DigitalInput;
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
	private static final int RIGHT_CYTRON_DIR_PIN = 7;
	private static final int BELT_CYTRON_PWM_PIN = 11;
	private static final int BELT_CYTRON_DIR_PIN = 10;

	private static final int LEFT_ENCODER_PIN_A = 2;
	private static final int LEFT_ENCODER_PIN_B = 5;
	private static final int RIGHT_ENCODER_PIN_A = 3;
	private static final int RIGHT_ENCODER_PIN_B = 4;
	
	private static final int LEFT_SHORT_IR_PIN = 12;
	private static final int RIGHT_SHORT_IR_PIN = 13;
	
	/** PID Controller paramaters. */
	private static final double P_ROT = 0.017;
	private static final double I_ROT = 0.00001;
	private static final double D_ROT = 1.0;
	private static final double P_TRANS = 0.03;
	private static final double I_TRANS = 0.00003;
	private static final double D_TRANS = 0.5;
	private static final double MAX_SPEED = 0.20;//changeme
	private static final double MAX_INTEGRAL_ERROR = 2000.0;
	
	private static final double BELT_SPEED = 0.39;

	private static final long WALL_TIME_CONST_MILLIS = 1000;
	private static final long STUCK_TIME_CONST_MILLIS = 600;

	private static final double STUCK_THRESH_ANGULAR_DIST = 0.002;
	private static final double STUCK_THRESH_WHEEL_CONTROL = 0.08;
	
	private MapleComm comm;
	private Cytron leftWheel, rightWheel, belt;
	private Encoder leftEncoder, rightEncoder;
	private DigitalInput leftShortIR, rightShortIR;
	
	private BoundedQueue<Error> errorHistory;
	
	/**
	 * Targets, in radians and inches.
	 */
	private double angleTarget;
	private double distanceTarget;
	
	private double leftWheelControl;
	private double rightWheelControl;
	
	/** Time at which we started being stuck. */
	private long stuckTime;
	
	
	/**
	 * Current error, in radians and inches.
	 */
	private Error error;
	
	/**
	 * Current position, in radians and inches.
	 */
	private double angleCurrent;
	// private double positionCurrent;
	
	private long leftWallTime, rightWallTime;
	
	RobotController(String port) {
		this.leftWheel = new Cytron(LEFT_CYTRON_DIR_PIN, LEFT_CYTRON_PWM_PIN);
		this.rightWheel = new Cytron(RIGHT_CYTRON_DIR_PIN, RIGHT_CYTRON_PWM_PIN);
		this.belt = new Cytron(BELT_CYTRON_DIR_PIN, BELT_CYTRON_PWM_PIN);
		this.leftEncoder = new Encoder(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
		this.rightEncoder = new Encoder(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);
		this.leftShortIR = new DigitalInput(LEFT_SHORT_IR_PIN);
		this.rightShortIR = new DigitalInput(RIGHT_SHORT_IR_PIN);
		this.angleTarget = 0;
		this.distanceTarget = 0;
		this.angleCurrent = 0;
		updateError();
		this.errorHistory = new BoundedQueue<Error>();
		this.errorHistory.add(this.error);
		this.leftWallTime = this.rightWallTime = (long)0;
		
		this.stuckTime = 0;
		
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
					belt.setSpeed(0);
					comm.transmit();
				}
			}
		}));
	}
	
	void setUpComm() {
	    if(!NO_COMM) {
    		comm.registerDevice(leftWheel);
    		comm.registerDevice(rightWheel);
    		comm.registerDevice(belt);
    		comm.registerDevice(rightEncoder);
    		comm.registerDevice(leftEncoder);
    		comm.registerDevice(leftShortIR);
    		comm.registerDevice(rightShortIR);
    
    		comm.initialize();
	    }
	}
	
	void setTarget(double angle, double distance) {
		this.angleTarget = toAngle(angle);
		this.distanceTarget = distance;
		updateError();
		//errorHistory.clear();
		this.errorHistory.add(this.error);
	}
	
	void setRelativeTarget(double relAngle, double distance) {
		System.err.printf("SETTING NEW REL ANGLE TARGET: %.2f\n", relAngle);
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
	
	public boolean closeToWall() {
	    return (wallOnLeft() || wallOnRight());
	}
	
	public boolean wallOnLeft() {
		return !leftShortIR.getValue();
	}
	
	public boolean wallOnRight() {
		return !rightShortIR.getValue();
	}
	
	public boolean wallOnBothSides() {
		return detectedLeftWallRecently() && detectedRightWallRecently();
	}
	
	public boolean detectedLeftWallRecently() {
		return (time() - leftWallTime < WALL_TIME_CONST_MILLIS);
	}
	
	public boolean detectedRightWallRecently() {
		return (time() - rightWallTime < WALL_TIME_CONST_MILLIS);
	}
	
	public boolean stuck() {
		return stuckTime != 0 && (time() - stuckTime > STUCK_TIME_CONST_MILLIS);
	}
	
	public double getAngleError() {
		return error.angleError;
	}
	
	public double getDistanceError() {
		return error.distanceError;
	}
	
	/**
	 * Updates error fields, infrared, and
	 * current angle and position.
	 */
	void updateMotionData() {
		// Update data
		if(!NO_COMM) {
			comm.updateSensorData();
		}
		
		// Retrieve data
		SensorData data = new SensorData();
		data.leftWheelAngularSpeed = leftEncoder.getAngularSpeed();
		data.rightWheelAngularSpeed = rightEncoder.getAngularSpeed();
		data.leftWheelDeltaAngularDistance = -1.0*leftEncoder.getDeltaAngularDistance();
		data.rightWheelDeltaAngularDistance = rightEncoder.getDeltaAngularDistance();
		data.wallOnLeft = wallOnLeft();
		data.wallOnRight = wallOnRight();
		data.time = time();		
		
		// Store IR wall detection times.
		if(data.wallOnLeft) {
			leftWallTime = time();
		} if(data.wallOnRight) {
			rightWallTime = time();
		}
		
		// Detect if we are stuck.
		if((Math.abs(data.leftWheelDeltaAngularDistance) < STUCK_THRESH_ANGULAR_DIST
				&& Math.abs(leftWheelControl) > STUCK_THRESH_WHEEL_CONTROL)
			||  (Math.abs(data.rightWheelDeltaAngularDistance) < STUCK_THRESH_ANGULAR_DIST
				&& Math.abs(rightWheelControl) > STUCK_THRESH_WHEEL_CONTROL)) {
			if(0 == stuckTime) /* Getting stuck this iteration. */ {
				stuckTime = time();
			}
		} else /* Not stuck */ { 
			stuckTime = 0;
		}
		
		// Update error.
		//double encoderAngularSpeed = (data.rightWheelAngularSpeed - data.leftWheelAngularSpeed)
		//		* WHEEL_RADIUS_IN_INCHES / (2.0 * HALF_WHEEL_SEPARATION_IN_INCHES);
		double angleTraveled = (data.rightWheelDeltaAngularDistance - data.leftWheelDeltaAngularDistance)
				* WHEEL_RADIUS_IN_INCHES / (2.0 * HALF_WHEEL_SEPARATION_IN_INCHES);
		// Angle target is absolute.
		angleCurrent = toAngle(angleCurrent + angleTraveled);
		double distanceTraveled = (data.rightWheelDeltaAngularDistance + data.leftWheelDeltaAngularDistance) * WHEEL_RADIUS_IN_INCHES / 2.0;
		// Distance target is relative.
		distanceTarget = distanceTarget - distanceTraveled;
		
	    System.out.println("Left wheel traveled:\t" + data.leftWheelDeltaAngularDistance + "\nRight wheel traveled:\t" + data.rightWheelDeltaAngularDistance);

	    updateError();
		errorHistory.add(error);
		
		debug();
		
		// Return copy to prevent unexpected modification.
		//return new MotionData(motionData);
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
		
		// Cap integral term
		if(angleErrorIntegral > MAX_INTEGRAL_ERROR) {
			angleErrorIntegral = MAX_INTEGRAL_ERROR;
		} else if(angleErrorIntegral < -1.0 * MAX_INTEGRAL_ERROR) {
			angleErrorIntegral = -1.0 * MAX_INTEGRAL_ERROR;
		}
		if(distanceErrorIntegral > MAX_INTEGRAL_ERROR) {
			distanceErrorIntegral = MAX_INTEGRAL_ERROR;
		} else if(distanceErrorIntegral < -1.0 * MAX_INTEGRAL_ERROR) {
			distanceErrorIntegral = -1.0 * MAX_INTEGRAL_ERROR;
		}
		
		// Angular controller:
		double rTermMax = 0.7 * MAX_SPEED / HALF_WHEEL_SEPARATION_IN_INCHES;
		double rP = P_ROT * angleError;
		double rI = I_ROT * angleErrorIntegral;
		double rD = D_ROT * angleErrorDerivative;
		if(Math.abs(rP) > rTermMax) {
			rP = Math.signum(rP) * rTermMax;
		}
		if(Math.abs(rI) > rTermMax) {
			rI = Math.signum(rI) * rTermMax;
		}
		if(Math.abs(rD) > rTermMax) {
			rD = Math.signum(rD) * rTermMax;
		}
		double rotationalControl = rP + rI + rD;
		
		// Translational controller (depends on angle error):
		double tTermMax = 0.7 * MAX_SPEED;
		double tP = P_TRANS * distanceError;
		double tI = I_TRANS * distanceErrorIntegral;
		double tD = D_TRANS * distanceErrorDerivative;
		if(Math.abs(tP) > tTermMax) {
			tP = Math.signum(tP) * tTermMax;
		}
		if(Math.abs(tI) > tTermMax) {
			tI = Math.signum(tI) * tTermMax;
		}
		if(Math.abs(tD) > tTermMax) {
			tD = Math.signum(tD) * tTermMax;
		}
		double translationalControl = (tP + tI + tD)
				* 1.0 / (1.0 + 2 * Math.abs(angleError));

		leftWheelControl = translationalControl - rotationalControl * HALF_WHEEL_SEPARATION_IN_INCHES;
		rightWheelControl = translationalControl + rotationalControl * HALF_WHEEL_SEPARATION_IN_INCHES;
		
		// Cap control magnitude at MAX_SPEED.
		leftWheelControl = (Math.abs(leftWheelControl) > MAX_SPEED) ? Math.signum(leftWheelControl)*MAX_SPEED : leftWheelControl;
		rightWheelControl = (Math.abs(rightWheelControl) > MAX_SPEED) ? Math.signum(rightWheelControl)*MAX_SPEED : rightWheelControl;

		leftWheel.setSpeed(leftWheelControl);
		rightWheel.setSpeed(rightWheelControl);
		belt.setSpeed(BELT_SPEED);

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
			System.out.printf("\tdistanceError:\t%.5f\n", distanceError);
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
			System.out.println("\twallOnLeft:\t" + wallOnLeft());
			System.out.println("\twallOnRight:\t" + wallOnRight());
		}
	}
	
	private static long time() {
		return System.currentTimeMillis();
	}
	
	private static class Error {
		double angleError, distanceError;
		long time;
		
		Error(double angleError, double distanceError) {
			this.angleError = angleError;
			this.distanceError = distanceError;
			this.time = time();
		}
		
		public String toString() {
		    return "Angle error: " + angleError + "\n" + "Distance error: " + distanceError;
		}
	}
}
