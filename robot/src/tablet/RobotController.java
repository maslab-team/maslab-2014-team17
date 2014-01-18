package tablet;

import java.awt.Point;

import comm.*;
import devices.sensors.Gyroscope;
import jssc.SerialPort;
import jssc.SerialPortException;
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
 * 	2) Call getSensorData() to update sensor data history in the robot
 * 	   controller and to access the recent data.
 * 	3) Call any combination of setXTarget functions.
 *  4) Call sendControl() to send the new instructions to the Maple.
 *  5) Repeat from step 2).
 * 
 * @author vmayar
 *
 */
public class RobotController {
	
	private static final int LEFT_CYTRON_PWM_PIN = 1;
	private static final int LEFT_CYTRON_DIR_PIN = 2;
	private static final int RIGHT_CYTRON_PWM_PIN = 6;
	private static final int RIGHT_CYTRON_DIR_PIN = 7;

	private static final int GYROSCOPE_SPI_PORT = 1;
	private static final int GYROSCOPE_SS_PIN = 9;
	private static final int LEFT_ENCODER_PIN_A = 0;
	private static final int LEFT_ENCODER_PIN_B = 0;
	private static final int RIGHT_ENCODER_PIN_A = 0;
	private static final int RIGHT_ENCODER_PIN_B = 0;
	
	
	private MapleComm comm;
	private Cytron leftWheel, rightWheel;
	private Encoder leftEncoder, rightEncoder;
	private Gyroscope gyroscope;
	
	private SensorDataHistory sensorHistory;
	private SensorData averagedSensorData;
	
	RobotController(String port) {
		this.comm = new MapleComm(MapleIO.SerialPortType.LINUX);
		this.leftWheel = new Cytron(LEFT_CYTRON_DIR_PIN, LEFT_CYTRON_PWM_PIN);
		this.rightWheel = new Cytron(RIGHT_CYTRON_DIR_PIN, RIGHT_CYTRON_PWM_PIN);
		this.gyroscope = new Gyroscope(GYROSCOPE_SPI_PORT, GYROSCOPE_SS_PIN);
		this.leftEncoder = new Encoder(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
		this.rightEncoder = new Encoder(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);
		this.sensorHistory = new SensorDataHistory();
		setUpComm();
	}
	
	private void setUpComm() {
		comm.registerDevice(leftWheel);
		comm.registerDevice(rightWheel);
		comm.registerDevice(gyroscope);
		comm.registerDevice(leftEncoder);
		comm.registerDevice(rightEncoder);

		comm.initialize();
	}
	
	void setPositionTarget(Point target) {
		
	}
	
	void setAngleTarget(int target) {
		
	}
	
	void setVelocityTarget(Point target) {
		
	}
	
	void setAngularVelocityTarget(int target) {
		
	}
	
	/**
	 * Returns a useful average of recent sensor data.  Adds the current
	 * sensor data to sensorHistory, and stores the average in averagedSensorData.
	 */
	SensorData getSensorData() {
		// Update data
		comm.updateSensorData();
		
		// Retrieve data
		SensorData data = new SensorData();
		data.gyroAngle = gyroscope.getTheta();
		data.gyroAngularSpeed = gyroscope.getOmega();
		data.leftWheelAngularSpeed = leftEncoder.getAngularSpeed();
		data.leftWheelAngularSpeed = rightEncoder.getAngularSpeed();
		
		// Add data to history
		sensorHistory.add(data);
		
		// Store new averages.
		averagedSensorData = averageSensorHistory();
		
		// Return copy to prevent unexpected modification.
		return new SensorData(averagedSensorData);
	}
	
	/**
	 * Returns a weighted average of sensor data,
	 * with recent data points weighted more heavily.
	 * 
	 * @return Weighted average of the sensor data
	 * in sensorHistory.
	 */
	private SensorData averageSensorHistory() {
		SensorData average = new SensorData();
		double gyroAngle = 0, gyroAngularSpeed = 0,
				leftWheelAngularSpeed = 0, rightWheelAngularSpeed = 0;
		int dataPointCtr = 1, dataPointTotal = 0;
		
		for(SensorData data : sensorHistory) { // Use iterative for loop for more efficient iteration.
			gyroAngle += dataPointCtr * data.gyroAngle;
			gyroAngularSpeed += dataPointCtr * data.gyroAngularSpeed;
			leftWheelAngularSpeed += dataPointCtr * data.leftWheelAngularSpeed;
			rightWheelAngularSpeed += dataPointCtr * data.rightWheelAngularSpeed;
			dataPointTotal += dataPointCtr;
			++dataPointCtr;
		}
		
		average.gyroAngle = gyroAngle / dataPointTotal;
		average.gyroAngularSpeed = gyroAngularSpeed / dataPointTotal;
		average.leftWheelAngularSpeed = leftWheelAngularSpeed / dataPointTotal;
		average.rightWheelAngularSpeed = rightWheelAngularSpeed / dataPointTotal;
		return average;
	}
	
	/**
	 * Submits the current instructions to the robot.
	 */
	void sendControl() {
		comm.transmit();
	}
}
