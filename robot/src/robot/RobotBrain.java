package robot;

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
	
	private static final boolean SPEAK = true;
	private static final int SPEAK_DELAY_MILLIS = 5000;
	private static final boolean DEBUG = true;
	private static final int SLEEP_TIME_MILLIS = 5;
	
	private RobotEye eye;
	private RobotController controller;
	private RobotWorld world;
	
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
	public RobotBrain(RobotEye eye, RobotController controller, RobotWorld world, long startTime) {
		this.eye = eye;
		this.controller = controller;
		this.controller.setCurrentAngle(world.getMap().startPose.theta);
		//this.controller.setCurrentPosition(world.getMap().startPose.x, world.getMap().startPose.y);
		this.world = world;
		//this.positionTarget = new Point(world.getMap().startPose.x, world.getMap().startPose.y);
		this.angleTarget = world.getMap().startPose.theta;
		this.distanceTarget = 0;
		this.startTime = startTime;
		this.timeMarker = 0;
		this.counter = 0;
		this.eyeData = new RobotEye.Data();
		this.startLooking();
		if(SPEAK) {
			speakTimeMarker = 0;
			mouth = new RobotMouth();
			strategy = "I'm getting ready to roomba.";
		}

		controller.setRelativeTarget(0.5, 0.0);
	}
	//findme
	private void startLooking() {
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
	
	private void stopLooking() {
		eyeThread.interrupt();
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
		
		/** Update every 5 seconds. */
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
		strategy = "Time for dessert - I'm going to find some red balls.";
	}
	
	void loop() {
		this.elapsedTime = System.currentTimeMillis() - startTime;
		++counter;
		updateWorld();
		speak();
		MotionData mData = controller.getMotionData();
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
	private double pixelToAngle(int pixelX) {
		int distFromCenter = RobotEye.IMAGE_WIDTH/2 - pixelX;
		return Math.atan2(distFromCenter, RobotEye.IMAGE_DEPTH_IN_PIXELS);
		
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