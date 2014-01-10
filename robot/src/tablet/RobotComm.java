package tablet;

import jssc.SerialPort;
import jssc.SerialPortException;

/**
 * RobotComm provides an abstraction layer around the
 * interface between the Maple and the robot's processor.
 * It contains methods to instruct the Maple to do
 * useful things.
 * 
 * Upon instantiation, sets up the serial port connection.
 * 
 * Remember to call close() after use to close connections.
 * 
 * @author vmayar
 *
 */
public class RobotComm {
	private SerialPort serialPort;
	
	RobotComm() {
		try {
			serialPort = new SerialPort("PORT");
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
        }
        catch (SerialPortException ex){
            System.out.println(ex);
        }
	}
	
	void close() {
		try {
			serialPort.closePort();
		} catch (SerialPortException ex) {
			System.out.println(ex);
		}
	}
	
	
}
