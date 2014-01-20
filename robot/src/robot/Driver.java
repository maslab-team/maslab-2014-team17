package robot;

import comm.*;
import devices.actuators.Cytron;
import devices.sensors.Gyroscope;

public class Driver {
    private MapleComm comm;
    private Cytron leftWheel, rightWheel;
    private Gyroscope gyro;
    
    public Driver() {
        comm = new MapleComm(MapleIO.SerialPortType.LINUX);
        leftWheel = new Cytron(2, 1);
        rightWheel = new Cytron(7, 6);
        gyro = new Gyroscope(1, 9);
    }
    
    void setup() {
        comm.registerDevice(leftWheel);
        comm.registerDevice(rightWheel);
        comm.registerDevice(gyro);
        comm.initialize();
    }
    
    void drive(double leftSpeed, double rightSpeed) {
        leftWheel.setSpeed(leftSpeed);
        rightWheel.setSpeed(rightSpeed);
        comm.transmit();
    }
    
    void updateReading() {
        comm.updateSensorData();
        System.out.println("omega: " + gyro.getOmega());
        System.out.println("theta: " + gyro.getTheta());
    }
    
    void rotateToAngle(double theta) throws InterruptedException {
        double currentAngle = gyro.getTheta();
        while(Math.abs(currentAngle - theta) > 0.1) {
            drive(-.03*(currentAngle - theta), .03*(currentAngle - theta));
            this.updateReading();
            currentAngle = gyro.getTheta();
            Thread.sleep(100);
        }
    }
    
    
    public static void main(String[] args) throws InterruptedException {
        Driver driver = new Driver();
        driver.setup();
        //driver.drive(0, 0);
        driver.rotateToAngle(1);
        
    }
}
