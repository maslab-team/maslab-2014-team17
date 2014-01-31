package tablet;

<<<<<<< HEAD
import comm.*;
import devices.sensors.Gyroscope;
import jssc.SerialPort;
import jssc.SerialPortException;

public class RobotController {
    
    public static void main(String[] args) throws SerialPortException {
        
        MapleComm comm = new MapleComm(MapleIO.SerialPortType.LINUX);
        comm.registerDevice(new Gyroscope(11, 10));
        comm.initialize();
        comm.transmit();
        comm.updateSensorData();
//        SerialPort serialPort;
//        try {
//            serialPort = new SerialPort("/dev/tty.usbmodemfa131");
//            serialPort.openPort();
//            serialPort.setParams(115200, 8, 1, 0);
//            
//            byte[] bytes;
//            
//            byte[] packet = buildPacket((byte) 'I', new byte[] {(byte) 'G'}, (byte) 0xff);
//            serialPort.writeBytes(packet);
//            while(true) {
//                bytes = serialPort.readBytes();
//                if(bytes != null) {
//                    for(byte elt : bytes) {
//                        System.out.print(elt + " ");
//                    }
//                    System.out.print('\n');
//                }
//            }
//        }
//        catch (SerialPortException ex){
//            System.out.println(ex);
//        }
    }
    
//    private static byte[] buildPacket(byte first, byte[] message, byte last) {
//        int len = message.length;
//        byte[] packet = new byte[len + 2];
//        packet[0] = first;
//        System.arraycopy(message, 0, packet, 1, len);
//        packet[len + 1] = last;
//        return packet;
//    }

=======
import comm.MapleComm;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;

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
		this.comm = new MapleComm(port);
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
	
>>>>>>> 1fdf37ab088dc4ebacefe2de83217b2ac7ec2c2b
}
