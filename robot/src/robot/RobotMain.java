package robot;

import BotClient.BotClient;
import model.RobotWorld;

public class RobotMain {
	
	private static final String maplePort = "/dev/tty.usbmodemfa131";
	
	public static void main(String[] args) {
		long startTime = System.currentTimeMillis();
		BotClient client = null;

		RobotController controller = new RobotController(maplePort);
		RobotBrain brain = new RobotBrain(controller, client, startTime);
		
		brain.setup();
		
		while(true) {
			brain.loop();
		}
	}

}
