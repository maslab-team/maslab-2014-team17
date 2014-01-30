package robot;

import comm.*;
import devices.actuators.Cytron;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Infrared;

public class Driver {
    private MapleComm comm;
    private Cytron leftWheel, rightWheel;
    private Encoder leftEncoder, rightEncoder;
    private Infrared ir;
    private PWMOutput servo;
    
    public Driver() {
        comm = new MapleComm(MapleIO.SerialPortType.LINUX);
        leftWheel = new Cytron(8, 9);
        rightWheel = new Cytron(5, 6);
        ir = new Infrared(0);
        servo = new PWMOutput(10);
        leftEncoder = new Encoder(3, 4);
        rightEncoder = new Encoder(2, 1);
    }
    
    void setup() {
        comm.registerDevice(leftWheel);
        comm.registerDevice(rightWheel);
        comm.registerDevice(leftEncoder);
        comm.registerDevice(rightEncoder);
        comm.registerDevice(ir);
        comm.registerDevice(servo);
        comm.initialize();
    }
    
    void drive(double leftSpeed, double rightSpeed) {
        leftWheel.setSpeed(leftSpeed);
        rightWheel.setSpeed(rightSpeed);
        comm.transmit();
    }
    
    void turn(double angle) {
        servo.setValue(angle);
    }
    
    float getDistance() {
        comm.updateSensorData();
        return ir.getDistance();
    }
    
    void getEncoderData() {
        System.out.println("left: " + leftEncoder.getAngularSpeed());
        System.out.println("right: " + rightEncoder.getAngularSpeed());
    }
    
    public static void main(String[] args) throws InterruptedException {
        Driver driver = new Driver();
        driver.setup();
        driver.drive(.2, -.2);
        driver.turn(.3);
        for(int i = 0; i < 10; i++) {
            float count = 0;
            for(int j = 0; j < 10; j++) {
                count += driver.getDistance();
            }
            count = count/10;
            System.out.println("ir: " + count);
            Thread.sleep(100);
            driver.getEncoderData();
        }
        driver.turn(-.3);
        driver.drive(0, 0);
    }
}
