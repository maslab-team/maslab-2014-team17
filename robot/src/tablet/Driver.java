package tablet;

import comm.*;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;

public class Driver {
    private MapleComm comm;
    private Cytron leftWheel, rightWheel;
    private Gyroscope gyro;
    private Encoder leftEncoder, rightEncoder;
    
    public Driver() {
        comm = new MapleComm(MapleIO.SerialPortType.LINUX);
        leftWheel = new Cytron(2, 1);
        rightWheel = new Cytron(7, 6);
        //gyro = new Gyroscope(13, 9);
        leftEncoder = new Encoder(36, 35);
        rightEncoder = new Encoder(34, 33);
    }
    
    void setup() {
        //comm.registerDevice(leftWheel);
        //comm.registerDevice(rightWheel);
        comm.registerDevice(leftEncoder);
        comm.registerDevice(rightEncoder);
        //comm.registerDevice(gyro);
        comm.initialize();
    }
    
    void drive(double leftSpeed, double rightSpeed) {
        leftWheel.setSpeed(leftSpeed);
        rightWheel.setSpeed(rightSpeed);
        comm.transmit();
    }
    
    void updateReading() {
        comm.updateSensorData();
        System.out.println("left: " + leftEncoder.getTotalAngularDistance());
        System.out.println("right: " + rightEncoder.getTotalAngularDistance());
    }
    
    void rotateToAngle(double theta) throws InterruptedException {
        double currentAngle = gyro.getTheta();
        while(Math.abs(currentAngle - theta) > 0.1) {
            drive(-.05*(currentAngle - theta), .05*(currentAngle - theta));
            this.updateReading();
            currentAngle = gyro.getTheta();
            Thread.sleep(100);
        }
    }
    
    
    public static void main(String[] args) throws InterruptedException {
        Driver driver = new Driver();
        driver.setup();
        //driver.drive(0.2, 0.2);
        for(int i = 0; i < 50; i++) {
            driver.updateReading();
            Thread.sleep(100);
        }
        driver.drive(0, 0);
    }
}
