package robot;

import java.util.Iterator;

import robot.datautils.BoundedQueue;
import robot.datautils.MotionData;
import robot.datautils.SensorData;
import robot.datautils.SensorDataHistory;

import comm.MapleComm;

import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;

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
	
	private static final boolean DEBUG = true;
	
	/** Wheel constants. */
	private static final double WHEEL_SEPARATION_IN_INCHES = 8.0;
	private static final double HALF_WHEEL_SEPARATION_IN_INCHES = WHEEL_SEPARATION_IN_INCHES / 2.0;
	private static final double WHEEL_DIAMETER_IN_INCHES = 3.88;
	private static final double WHEEL_RADIUS_IN_INCHES = WHEEL_DIAMETER_IN_INCHES / 2.0;
	
	private static final int LEFT_CYTRON_PWM_PIN = 1;
	private static final int LEFT_CYTRON_DIR_PIN = 2;
	private static final int RIGHT_CYTRON_PWM_PIN = 6;
	private static final int RIGHT_CYTRON_DIR_PIN = 7;

	private static final int GYROSCOPE_SPI_PORT = 1;
	private static final int GYROSCOPE_SS_PIN = 9;
	
	private static final int LEFT_ENCODER_PIN_A = 36;
	private static final int LEFT_ENCODER_PIN_B = 35;
	private static final int RIGHT_ENCODER_PIN_A = 34;
	private static final int RIGHT_ENCODER_PIN_B = 33;
	
	/** PID Controller paramaters. */
	private static final double P_ROT = 0.5;
	private static final double I_ROT = 0;
	private static final double D_ROT = 0;
	private static final double P_TRANS = 0.5;
	private static final double I_TRANS = 0;
	private static final double D_TRANS = 0;

	
	
	private MapleComm comm;
	private Cytron leftWheel, rightWheel;
	private Encoder leftEncoder, rightEncoder;
	private Gyroscope gyroscope;
	
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
		this.comm = new MapleComm(port);
		this.leftWheel = new Cytron(LEFT_CYTRON_DIR_PIN, LEFT_CYTRON_PWM_PIN);
		this.rightWheel = new Cytron(RIGHT_CYTRON_DIR_PIN, RIGHT_CYTRON_PWM_PIN);
		this.gyroscope = new Gyroscope(GYROSCOPE_SPI_PORT, GYROSCOPE_SS_PIN);
		this.leftEncoder = new Encoder(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
		this.rightEncoder = new Encoder(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);
		this.sensorHistory = new SensorDataHistory();
		this.angleTarget = 0;
		this.distanceTarget = 0;
		this.angleCurrent = 0;
		updateError();
		this.errorHistory = new BoundedQueue<Error>();
		this.errorHistory.add(this.error);
		setUpComm();
	}
	
	/**
	 * Sets the current orientation of the robot.
	 * 
	 * @param angleInRadians
	 */
	void setCurrentAngle(double angleInRadians) {
		this.angleCurrent = angleInRadians;
		updateError();
	}
	
	private void updateError() {
		this.error = new Error(this.angleTarget - this.angleCurrent, this.distanceTarget);
	}
	
	/*TODO:
	void setCurrentPosition(Point position) {
		
	}*/
	
	private void setUpComm() {
		comm.registerDevice(leftWheel);
		comm.registerDevice(rightWheel);
		comm.registerDevice(gyroscope);
		comm.registerDevice(leftEncoder);
		comm.registerDevice(rightEncoder);

		comm.initialize();
	}
	
	void setTarget(double angle, double distance) {
		this.angleTarget = angle;
		this.distanceTarget = distance;
		updateError();
		errorHistory.clear();
		this.errorHistory.add(this.error);
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
		comm.updateSensorData();
		
		// Retrieve data
		SensorData data = new SensorData();
		data.gyroAngle = gyroscope.getTheta();
		data.gyroAngularSpeed = gyroscope.getOmega();
		data.leftWheelAngularSpeed = leftEncoder.getAngularSpeed();
		data.rightWheelAngularSpeed = rightEncoder.getAngularSpeed();
		data.leftWheelDeltaAngularDistance = leftEncoder.getDeltaAngularDistance();
		data.rightWheelDeltaAngularDistance = rightEncoder.getDeltaAngularDistance();
		data.time = System.currentTimeMillis();
		
		// Add data to history
		sensorHistory.add(data);
		
		// Store new averages.
		motionData = sensorHistory.getMotionData();
		
		// Update error.
		//double encoderAngularSpeed = (data.rightWheelAngularSpeed - data.leftWheelAngularSpeed)
		//		* WHEEL_RADIUS_IN_INCHES / (WHEEL_SEPARATION_IN_INCHES / 2.0);
		double angleTraveled = (data.rightWheelDeltaAngularDistance - data.leftWheelDeltaAngularDistance)
				* WHEEL_RADIUS_IN_INCHES / HALF_WHEEL_SEPARATION_IN_INCHES;
		// Angle target is absolute.
		angleCurrent = angleCurrent - angleTraveled;
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
		
		double angleErrorDerivative = 0, angleErrorIntegral = 0;
		double distanceErrorDerivative = 0, distanceErrorIntegral = 0;
		double angleError = error.angleError, distanceError = error.distanceError;
		// Calculate controller terms.
		Iterator<Error> itrFront = errorHistory.iterator();
		Iterator<Error> itrBack = errorHistory.iterator();
		if(!itrFront.hasNext()) {
			throw new RuntimeException("No error recorded?");
		}
		Error front = itrFront.next();
		Error back = null;
		while(itrFront.hasNext()) {
			front = itrFront.next();
			back = itrBack.next();
			
			angleErrorDerivative += weight * (front.angleError - back.angleError)
					/ (front.time - back.time);
			distanceErrorDerivative += weight * (front.distanceError - back.distanceError)
					/ (front.time - back.time);
			
			angleErrorIntegral += (front.angleError - back.angleError) * (front.time - back.time);
			angleErrorIntegral += (front.angleError - back.angleError) * (front.time - back.time);
			

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
		
		// Cap control magnitude at 1.0.
		leftWheelControl = (Math.abs(leftWheelControl) > 1.0) ? Math.signum(leftWheelControl) : leftWheelControl;
		rightWheelControl = (Math.abs(rightWheelControl) > 1.0) ? Math.signum(rightWheelControl) : rightWheelControl;

		leftWheel.setSpeed(leftWheelControl);
		rightWheel.setSpeed(rightWheelControl);
		
		if(DEBUG) {
			System.out.printf("Rotational control:\n");
			System.out.printf("\tP_ROT:%.2f\n", P_ROT);
			System.out.printf("\tI_ROT:%.2f\n", I_ROT);
			System.out.printf("\tD_ROT:%.2f\n", D_ROT);
			System.out.printf("\tangleError:%.2f\n", angleError);
			System.out.printf("\tangleErrorIntegral:%.2f\n", angleErrorIntegral);
			System.out.printf("\tangleErrorDerivative:%.2f\n", angleErrorDerivative);
			System.out.printf("\trotational control:%.2f\n", rotationalControl);
			System.out.printf("Translational control:\n");
			System.out.printf("\tP_ROT:%.2f\n", P_TRANS);
			System.out.printf("\tI_ROT:%.2f\n", I_TRANS);
			System.out.printf("\tD_ROT:%.2f\n", D_TRANS);
			System.out.printf("\tangleError:%.2f\n", angleError);
			System.out.printf("\tdistanceErrorIntegral:%.2f\n", distanceErrorIntegral);
			System.out.printf("\tdistanceErrorDerivative:%.2f\n", distanceErrorDerivative);
			System.out.printf("\ttranslational control:%.2f\n", translationalControl);
			System.out.printf("Left wheel control:%.2f\n", leftWheelControl);
			System.out.printf("Right wheel control:%.2f\n", rightWheelControl);

			debug();
		}
		
		comm.transmit();
	}
	
	private void debug() {
		System.out.printf("Controller info:\n");
		System.out.printf("\tangleError:\n", error.angleError);
		System.out.printf("\tdistanceError:\n", error.distanceError);
		System.out.printf("\tangleCurrent:\n", angleCurrent);
		System.out.printf("\tangleTarget:\n", angleTarget);
		System.out.printf("\tangleTarget:\n", distanceTarget);
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
