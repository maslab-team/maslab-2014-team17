package tablet;

import comm.*;
import devices.actuators.Cytron;

public class Driver {
    private MapleComm comm;
    private Cytron leftWheel, rightWheel;
    
    public Driver() {
        comm = new MapleComm(MapleIO.SerialPortType.LINUX);
        leftWheel = new Cytron(2, 1);
        rightWheel = new Cytron(7, 6);
    }
    
    void setup() {
        comm.registerDevice(leftWheel);
        comm.registerDevice(rightWheel);
        comm.initialize();
    }
    
    void drive(double leftSpeed, double rightSpeed) {
        leftWheel.setSpeed(leftSpeed);
        rightWheel.setSpeed(rightSpeed);
        comm.transmit();
    }
    
    public static void main(String[] args) {
        Driver driver = new Driver();
        driver.setup();
        driver.drive(2.0, 2.0);
    }
}
