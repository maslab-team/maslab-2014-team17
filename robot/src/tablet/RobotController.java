package tablet;

import comm.*;
import devices.sensors.Gyroscope;
import jssc.SerialPort;
import jssc.SerialPortException;
import devices.actuators.Cytron;
import devices.sensors.Encoder;

public class RobotController {
	
	private static final int LEFT_CYTRON_PWM_PIN = 0;
	private static final int LEFT_CYTRON_DIR_PIN = 0;
	private static final int RIGHT_CYTRON_PWM_PIN = 0;
	private static final int RIGHT_CYTRON_DIR_PIN = 0;

	private static final int GYROSCOPE_SPI_PORT = 0;
	private static final int GYROSCOPE_SS_PIN = 0;
	private static final int LEFT_ENCODER_PIN_A = 0;
	private static final int LEFT_ENCODER_PIN_B = 0;
	private static final int RIGHT_ENCODER_PIN_A = 0;
	private static final int RIGHT_ENCODER_PIN_B = 0;
	
	
	private MapleComm comm;
	private Cytron leftWheel, rightWheel;
	private Encoder leftEncoder, rightEncoder;
	private Gyroscope gyroscope;
	
	RobotController(String port) {
		this.comm = new MapleComm(MapleIO.SerialPortType.LINUX);
		this.leftWheel = new Cytron(LEFT_CYTRON_DIR_PIN, LEFT_CYTRON_PWM_PIN);
		this.rightWheel = new Cytron(RIGHT_CYTRON_DIR_PIN, RIGHT_CYTRON_PWM_PIN);
		this.gyroscope = new Gyroscope(GYROSCOPE_SPI_PORT, GYROSCOPE_SS_PIN);
		this.leftEncoder = new Encoder(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
		this.rightEncoder = new Encoder(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);
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
	
	void setPositionTarget() {
		
	}
	
	void setAngleTarget() {
		
	}
	
	void setVelocityTarget() {
		
	}
	
	void setAngularVelocityTarget() {
		
	}
	
	/*
	 * TODO: Retrieve, store, and return sensor data.
	 */
	void getSensorData() {
		comm.updateSensorData();
	}
	
	/**
	 * Submits the current instructions to the robot.
	 */
	void sendControl() {
		comm.transmit();
	}
}
