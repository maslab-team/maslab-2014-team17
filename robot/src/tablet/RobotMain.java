package tablet;

public class RobotMain {

	public static void main(String[] args) {
		long startTime = System.currentTimeMillis();
		
		RobotComm comm = new RobotComm();
		RobotEye eye = new RobotEye();
		RobotBrain brain = new RobotBrain(eye, comm, startTime);
		
		while(true) {
			brain.loop();
		}
	}

}
