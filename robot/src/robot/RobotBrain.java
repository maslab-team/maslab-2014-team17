package robot;

import BotClient.BotClient;
import model.RobotWorld;
import robot.datautils.MotionData;

/**
 * Contains method necessary for processing inputs
 * and making decisions.
 * 
 * @author vmayar
 *
 */
public class RobotBrain {
	
	private static final boolean SPEAK = false;
	private static final int SPEAK_DELAY_MILLIS = 5000;
	private static final boolean LOOK = false;
	private static final int CAMERA_NUMBER = 0;
	private static final boolean DEBUG = false;

	private static final int SLEEP_TIME_MILLIS = 5;
	private static final boolean USE_BOTCLIENT = false;
	
	private static final int PING_PONG_DELAY_MILLIS = 50;
	private static final int STUCK_DELAY_MILLIS = 1200;
	
	private RobotEye eye;
	private RobotController controller;
	private RobotWorld world;
	private BotClient client;
	
	/**
	 * Holds data from the eye.
	 */
	private RobotEye.Data eyeData;
	
	/**
	 * Lock used to control modification (esp. reassignment)
	 *  of eyeData.
	 */
	private Object eyeDataLock = new Object();
	
	/** Indicates whether there is new data from the eye. */
	private volatile boolean newEyeData = false;
	
	/** Mouth */
	private RobotMouth mouth;
	
	/** Strategy (for speaking) */
	private String strategy;
	
	private long speakTimeMarker;
	private long stuckTimeMarker;
	
	/*/** Represents position target in xy-plane.
	private Point positionTarget;*/
	
	/**
	 * Represents angular position target in radians.
	 * Counterclockwise direction is positive.
	 */
	private double angleTarget;
	
	/**
	 * Distance to target.
	 */
	private double distanceTarget;
	
	/**
	 * Start time of the game, in millis since epoch.
	 */
	private long startTime;
	
	/**
	 * Time elapsed since game start.
	 */
	private long elapsedTime;
	
	private long timeMarker;

	/**
	 * Iterations since game start.
	 */
	private int counter;
	
	/**
	 * Thread that runs vision processing.
	 */
	private Thread eyeThread;
	
	/**
	 * Create a new brain with a given eye, controller, and initial world.
	 * 
	 * @param eye
	 * @param controller
	 * @param world
	 * @param startTime
	 */
	public RobotBrain(RobotController controller,
			BotClient client, long startTime) {
		if (LOOK) {
			this.eye = new RobotEye(CAMERA_NUMBER);
		}
		this.controller = controller;
		//this.controller.setCurrentPosition(world.getMap().startPose.x, world.getMap().startPose.y);
		//this.positionTarget = new Point(world.getMap().startPose.x, world.getMap().startPose.y);
		this.distanceTarget = 0;
		this.startTime = startTime;
		this.timeMarker = (long)0;
		this.stuckTimeMarker = (long)0;
		this.counter = 0;
		this.eyeData = new RobotEye.Data();
		if(SPEAK) {
			speakTimeMarker = (long)0;
			mouth = new RobotMouth();
			strategy = "I'm getting ready to roomba.";
		}
		if(USE_BOTCLIENT) {
			this.client = new BotClient("18.150.7.174:6667","1221",false);
		}
	}

	private void startLooking() {
		if (LOOK) {
			eyeThread = new Thread(new Runnable() {
				public void run() {
					while(true) {
						RobotEye.Data data = eye.process();
						synchronized(eyeDataLock) {
							eyeData = data;
							newEyeData = true;
						}
					}
				}
			});
			eyeThread.start();
		}
	}
	
	private void stopLooking() {
		if(LOOK) { eyeThread.interrupt(); }
	}
	
	private void updateWorld() {
		RobotEye.Data data;
		if(newEyeData) {
			newEyeData = false;
			synchronized(eyeDataLock) {
				data = eyeData;
			}
			world.update(data);
		}
	}
	
	private void speak() {
		if(SPEAK && elapsedTime - speakTimeMarker > SPEAK_DELAY_MILLIS) {
			mouth.speak(strategy);
			speakTimeMarker = elapsedTime;
		}
	}

	private void goToGreenBalls() {
		
		/* Update every 5 seconds. */
		if(elapsedTime - timeMarker > 5000) {

			RobotWorld.Ball greenBall = world.getLargestGreenBall();
			if(greenBall != null) {
				strategy = "I found a green ball! I'm going to go eat it.";
				angleTarget = pixelToAngle(greenBall.getX());
				timeMarker = elapsedTime;
				//TODO: distanceTarget = radiusToDistance(largestBall.getRadius());
			} else {
				strategy = "I'm trying to find some delicious green balls to eat.";
			}

			controller.setRelativeTarget(angleTarget, distanceTarget);
		}
	}
	
	private void depositBalls() {
		strategy = "I ate too many green balls.  I'm going to put some in the reactors.";
	}
	
	private void goToRedBalls() {
		/* Update every 3 seconds. */
		if(elapsedTime - timeMarker > 3000) {

			RobotWorld.Ball redBall = world.getLargestRedBall();
			if(redBall != null) {
				strategy = "I found a red ball! I'm going to go eat it.";
				angleTarget = pixelToAngle(redBall.getX());
				distanceTarget = 5;
				timeMarker = elapsedTime;
				controller.setRelativeTarget(angleTarget, distanceTarget);

				//TODO: distanceTarget = radiusToDistance(largestBall.getRadius());
			} else {
				strategy = "I'm trying to find some delicious red balls to eat.";
			}

		}
	}
	
	private void goToBalls() {
		RobotWorld.Ball ball = world.getLargestBall();
		if(ball != null) {
			strategy = "I found a " + ball.getColor() + "  ball! I'm going to go eat it.";
			angleTarget = pixelToAngle(ball.getX());
			timeMarker = elapsedTime;
			controller.setRelativeTarget(angleTarget, distanceTarget);
		} else {
			angleTarget = 0.0;
			strategy = "I'm trying to find some delicious balls to eat.";
		}
		distanceTarget = 12.0;
	}
	
	/** Ping pong strategy */
	private void pingPong() {
		if(elapsedTime - timeMarker > PING_PONG_DELAY_MILLIS
				&& elapsedTime - stuckTimeMarker > STUCK_DELAY_MILLIS) {
			if(controller.stuck()) {
				System.err.println("\nI KNOW I'M STUCK\n");
				//System.exit(0);
				stuckTimeMarker = elapsedTime;
				if(angleTarget > 0.0) {
					angleTarget = -0.5 * Math.PI/2.0;
				} else {
					angleTarget = 0.5 * Math.PI/2.0;
				}
				distanceTarget = -5.0;

			} else if(controller.wallOnBothSides()) {
				angleTarget = 0;
				angleTarget = -0.1 * Math.PI/2.0;
				distanceTarget = -5.0;
				stuckTimeMarker = elapsedTime;
			} else if(controller.wallOnLeft()){
				angleTarget = -0.3 * Math.PI/2.0;
				distanceTarget = -2.0;
			} else if(controller.wallOnRight()){
				angleTarget = 0.3 * Math.PI/2.0;
				distanceTarget = -2.0;
			} else if(!controller.closeToWall()) {
				goToBalls();
			} 
			controller.setRelativeTarget(angleTarget, distanceTarget);
			timeMarker = elapsedTime;
		}
	}
	
	public void setup() {
		// Wait for arduino to be ready.
		try {
			Thread.sleep(2000);
		} catch(InterruptedException e) {
			System.err.println(e);
		}
		
		// Wait for botclient.
		if(USE_BOTCLIENT) {
			System.out.println("Waiting for game to start...");
			while(!client.gameStarted());
			System.out.println("The game has started!");
		}
		//mouth.speak("The game has started!");
		
		this.world = new RobotWorld(client);
		this.controller.setCurrentAngle(world.getMap().startPose.theta);
		this.angleTarget = 0;//world.getMap().startPose.theta;
		this.startLooking();
		controller.setUpComm();
		 
        angleTarget = 0;
        distanceTarget = 0;
        controller.setRelativeTarget(angleTarget, distanceTarget);

	}
	
	void loop() {
		this.elapsedTime = time() - startTime;
		++counter;
		updateWorld();
		speak();
		MotionData mData = controller.getMotionData();

		pingPong();
		/*
		if(elapsedTime < 1000 * 60 * 1) {
			goToGreenBalls();			
		} else if (elapsedTime < 1000 * 60 * 2) {
			depositBalls();
		} else if (elapsedTime < 1000 * 60 * 3) {
			goToRedBalls();
		} else {
			stopLooking();
			strategy = "Game over.  I probably lost.";
			System.out.println("Game over.");
		}*/
		
		controller.sendControl();
		debug();

		try {
			Thread.sleep(SLEEP_TIME_MILLIS);
		} catch(InterruptedException ex) {
			ex.printStackTrace();
		}
		
	}
	
	/**
	 * Given a horizontal pixel coordinate, return the angle from
	 * the direction of the camera to the given coordinate.
	 * 
	 * @param pixelX
	 * @return
	 */
	private static double pixelToAngle(int pixelX) {
		int distFromCenter = RobotEye.IMAGE_WIDTH/2 - pixelX;
		return Math.atan2(distFromCenter, RobotEye.IMAGE_DEPTH_IN_PIXELS);
		
	}
	
	private static long time() {
		return System.currentTimeMillis();
	}
	
	private void debug() {
		if(DEBUG) {
			System.out.printf("ROBOT INFO\n");
			System.out.printf("\tTime:\t%d\n", elapsedTime);
			System.out.printf("\tCounter:\t%d\n", counter);
			//System.out.printf("\tPosition Target:\t" + positionTarget + "\n");
			System.out.printf("\tDistance Target:\t%.2f\n", distanceTarget);
			System.out.printf("\tAngle Target:\t%.2f\n", angleTarget);
		}
	}
	
}