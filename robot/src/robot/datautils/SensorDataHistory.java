package robot.datautils;

import java.util.Iterator;

/**
 * Keeps track of previous sensor data, with a limit on the number
 * of stored data points.  On add(), removes old elements, if the
 * size exceeds maxSize.
 * 
 * WARNING: Not thread-safe.
 * 
 * @author vmayar
 *
 */
public class SensorDataHistory extends BoundedQueue<SensorData>{
	
	private static final long serialVersionUID = -2505869499102386460L;
	
	/**
	 * Number of data points between points used to calculate
	 * acceleration and distance.
	 */
	private static final int DATA_POINT_GAP = 5;
	
	/**
	 * Returns a weighted average of sensor data,
	 * with recent data points weighted more heavily,
	 * along with other information about motion.
	 * 
	 * @return Motion data
	 */
	public MotionData getMotionData() {
		MotionData mData = new MotionData();
		double gyroAngle = 0, gyroAngularSpeed = 0,
				leftWheelAngularSpeed = 0, rightWheelAngularSpeed = 0;
		int dataPointCtr = 1, dataPointTotal = 0;
		
		Iterator<SensorData> itrFirst = this.iterator();
		Iterator<SensorData> itrSecond = this.iterator();
		for(int ctr = 0; ctr < DATA_POINT_GAP; ++ctr) {
			if(!itrFirst.hasNext()) {
				return mData;
			}
			itrFirst.next();
		}
		if(!itrFirst.hasNext()) {
			return mData;
		}
		SensorData first = itrFirst.next();
		SensorData second = itrSecond.next();
		
		while(itrFirst.hasNext()) { // Use iterative for loop for more efficient iteration.
			SensorData dataFirst = itrFirst.next();
			SensorData dataSecond = itrSecond.next();
			gyroAngle += dataPointCtr * dataFirst.gyroAngle;
			gyroAngularSpeed += dataPointCtr * dataFirst.gyroAngularSpeed;
			leftWheelAngularSpeed += dataPointCtr * dataFirst.leftWheelAngularSpeed;
			rightWheelAngularSpeed += dataPointCtr * dataFirst.rightWheelAngularSpeed;
			dataPointTotal += dataPointCtr;
			++dataPointCtr;
		}
		
		if(0 == dataPointTotal) {
			return mData;
		}
		
		mData.gyroAngle = gyroAngle / dataPointTotal;
		mData.gyroAngularSpeed = gyroAngularSpeed / dataPointTotal;
		mData.leftWheelAngularSpeed = leftWheelAngularSpeed / dataPointTotal;
		mData.rightWheelAngularSpeed = rightWheelAngularSpeed / dataPointTotal;
		return mData;
	}
	
}
