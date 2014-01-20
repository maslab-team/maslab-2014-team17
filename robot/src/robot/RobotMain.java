package robot;

import model.RobotWorld;

public class RobotMain {
	
	private static final String maplePort = "/dev/tty.usbmodemfa131";
	private static final int cameraNum = 0;
	
	public static void main(String[] args) {
		long startTime = System.currentTimeMillis();
		
		RobotController controller = new RobotController(maplePort);
		RobotEye eye = new RobotEye(cameraNum);
		RobotWorld world = new RobotWorld();
		RobotBrain brain = new RobotBrain(eye, controller, world, startTime);
		
		while(true) {
			brain.loop();
		}
	}

}
