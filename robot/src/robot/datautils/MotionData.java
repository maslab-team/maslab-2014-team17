package robot.datautils;

/**
 * Struct-like class for storing motion data.
 * All fields are public and can be modified
 * directly.
 * 
 * @author vmayar
 *
 */
public class MotionData {

	public double leftWheelAngularSpeed;
	public double rightWheelAngularSpeed;
	public double gyroAngularSpeed;
	public double gyroAngle;
	public double leftWheelAngularAcceleration;
	public double rightWheelAngularAcceleration;
	public double gyroAngularAcceleration;
	
	MotionData() {
		leftWheelAngularSpeed = 0;
		rightWheelAngularSpeed = 0;
		gyroAngularSpeed = 0;
		gyroAngle = 0;
		leftWheelAngularAcceleration = 0;
		rightWheelAngularAcceleration = 0;
		gyroAngularAcceleration = 0;
	}
	
	MotionData(double leftWheelAngularSpeed, double rightWheelAngularSpeed,
			double gyroAngularSpeed, double gyroAngle,
			double leftWheelAngularAcceleration, double rightWheelAngularAcceleration,
			double gyroAngularAcceleration) {
		this.leftWheelAngularSpeed = leftWheelAngularSpeed;
		this.rightWheelAngularSpeed = rightWheelAngularSpeed;
		this.gyroAngularSpeed = gyroAngularSpeed;
		this.gyroAngle = gyroAngle;
		this.leftWheelAngularAcceleration = leftWheelAngularAcceleration;
		this.rightWheelAngularAcceleration = rightWheelAngularAcceleration;
		this.gyroAngularAcceleration = gyroAngularAcceleration;
	}
	
	/**
	 * Create a new instance of MotionData with values equivalent
	 * to the original.  (Clones the original.)
	 * 
	 * @param original The instane of MotionData to clone.
	 */
	public MotionData(MotionData original) {
		this.leftWheelAngularSpeed = original.leftWheelAngularSpeed;
		this.rightWheelAngularSpeed = original.rightWheelAngularSpeed;
		this.gyroAngularSpeed = original.gyroAngularSpeed;
		this.gyroAngle = original.gyroAngle;
		this.leftWheelAngularAcceleration = original.leftWheelAngularAcceleration;
		this.rightWheelAngularAcceleration = original.rightWheelAngularAcceleration;
		this.gyroAngularAcceleration = original.gyroAngularAcceleration;
	}
	
	public String toString() {
		return "\tMotion Data:\n";
		
	}
	
}
