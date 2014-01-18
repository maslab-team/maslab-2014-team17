package tablet;

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
	
	SensorData() {
		leftWheelAngularSpeed = 0;
		rightWheelAngularSpeed = 0;
		gyroAngularSpeed = 0;
		gyroAngle = 0;
	}
	
	SensorData(double leftWheelAngularSpeed, double rightWheelAngularSpeed,
			double gyroAngularSpeed, double gyroAngle) {
		this.leftWheelAngularSpeed = leftWheelAngularSpeed;
		this.rightWheelAngularSpeed = rightWheelAngularSpeed;
		this.gyroAngularSpeed = gyroAngularSpeed;
		this.gyroAngle = gyroAngle;
	}
	
	/**
	 * Create a new instance of SensorData with values equivalent
	 * to the original.  (Clones the original.)
	 * 
	 * @param original The instane of SensorData to clone.
	 */
	SensorData(SensorData original) {
		this.leftWheelAngularSpeed = original.leftWheelAngularSpeed;
		this.rightWheelAngularSpeed = original.rightWheelAngularSpeed;
		this.gyroAngularSpeed = original.gyroAngularSpeed;
		this.gyroAngle = original.gyroAngle;		
	}
	
}
