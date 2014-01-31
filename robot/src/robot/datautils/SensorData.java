package robot.datautils;

/**
 * Struct-like class for storing sensor data.
 * All fields are public and can be modified
 * directly.
 * 
 * @author vmayar
 *
 */
public class SensorData {

	public double leftWheelAngularSpeed;
	public double rightWheelAngularSpeed;
	public double gyroAngularSpeed;
	public double gyroAngle;
	public double leftWheelDeltaAngularDistance;
	public double rightWheelDeltaAngularDistance;
	public boolean wallOnLeft, wallOnRight;
	
	public long time;
	
	public SensorData() {
		leftWheelAngularSpeed = 0;
		rightWheelAngularSpeed = 0;
		gyroAngularSpeed = 0;
		gyroAngle = 0;
		wallOnLeft = wallOnRight = false;
		time = 0;
	}
	
	SensorData(double leftWheelAngularSpeed, double rightWheelAngularSpeed,
			double leftWheelDeltaAngularDistance, double rightWheelDeltaAngularDistance,
			double gyroAngularSpeed, double gyroAngle, boolean wallOnLeft, boolean wallOnRight, long time) {
		this.leftWheelAngularSpeed = leftWheelAngularSpeed;
		this.rightWheelAngularSpeed = rightWheelAngularSpeed;
		this.gyroAngularSpeed = gyroAngularSpeed;
		this.gyroAngle = gyroAngle;
		this.time = time;
		this.leftWheelDeltaAngularDistance = leftWheelDeltaAngularDistance;
		this.rightWheelDeltaAngularDistance = rightWheelDeltaAngularDistance;
		this.wallOnRight = wallOnRight;
		this.wallOnLeft = wallOnLeft;
	}
	
	/**
	 * Create a new instance of SensorData with values equivalent
	 * to the original.  (Clones the original.)
	 * 
	 * @param original The instance of SensorData to clone.
	 */
	SensorData(SensorData original) {
		this.leftWheelAngularSpeed = original.leftWheelAngularSpeed;
		this.rightWheelAngularSpeed = original.rightWheelAngularSpeed;
		this.gyroAngularSpeed = original.gyroAngularSpeed;
		this.gyroAngle = original.gyroAngle;		
		this.time = original.time;
		this.leftWheelDeltaAngularDistance = original.leftWheelDeltaAngularDistance;
		this.rightWheelDeltaAngularDistance = original.rightWheelDeltaAngularDistance;
		this.wallOnRight = original.wallOnRight;
		this.wallOnLeft = original.wallOnLeft;	}
	
}
