package robot;

import BotClient.BotClient;
import model.RobotWorld;

public class RobotMain {
	
	private static final String maplePort = "/dev/tty.usbmodemfa1331";
	private static final int cameraNum = 0;
	
	public static void main(String[] args) {
		long startTime = System.currentTimeMillis();
		BotClient client = null;

		RobotController controller = new RobotController(maplePort);
		RobotEye eye = new RobotEye(cameraNum);
		RobotBrain brain = new RobotBrain(eye, controller, client, startTime);
		
		brain.setup();
		
		while(true) {
			brain.loop();
		}
	}

}
