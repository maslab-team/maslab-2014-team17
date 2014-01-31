package robot;

import model.RobotWorld;
import BotClient.BotClient;

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
	private static final boolean LOOK = true;
	private static final int CAMERA_NUMBER = 0;
	private static final boolean DEBUG = true;

	private static final int SLEEP_TIME_MILLIS = 5;
	private static final boolean USE_BOTCLIENT = false;
	
	private static final int PING_PONG_DELAY_MILLIS = 100;
	private static final int STUCK_DELAY_MILLIS = 1500;
	private static final int BOT_CLIENT_DELAY_MILLIS = 377;
	
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
	private long botClientTimeMarker;
	
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
	private long ballTimeMarker;

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
		this.ballTimeMarker = (long)0;
		this.botClientTimeMarker = (long)0;
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
	
	private void sendStrategy() {
		if(USE_BOTCLIENT && elapsedTime - botClientTimeMarker > BOT_CLIENT_DELAY_MILLIS) {
			client.sendRaw(strategy);
			botClientTimeMarker = elapsedTime;
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
		if(elapsedTime - timeMarker > 1500) {

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
		if(elapsedTime - ballTimeMarker > 800) {
			RobotWorld.Ball ball = world.getLargestBall();//findme
			if(ball != null) {
				strategy = "I found a " + ball.getColor() + "  ball! I'm going to go eat it.";
				//angleTarget = pixelToAngle(ball.getX());
				angleTarget = controller.getAngleError()/3.0 + 2.0*pixelToAngle(ball.getX())/3.0;
				System.out.printf("Ball at " + ball.getX() + ". Angle: %.2f\n", angleTarget);
				ballTimeMarker = elapsedTime;
			} else {
				angleTarget = 0.0;
				strategy = "I'm trying to find some delicious balls to eat.";
			}
			distanceTarget = 12.0;
			controller.setRelativeTarget(angleTarget, distanceTarget);
		}
	}
	
	/** Ping pong strategy */
	private void pingPong() {
		if(elapsedTime - timeMarker > PING_PONG_DELAY_MILLIS
				&& elapsedTime - stuckTimeMarker > STUCK_DELAY_MILLIS) {
			if(controller.stuck()) {
				strategy = "I'm stuck.";
				stuckTimeMarker = elapsedTime;
				if(angleTarget > 0.0) {
					angleTarget = -0.5 * Math.PI/2.0;
				} else {
					angleTarget = 0.5 * Math.PI/2.0;
				}
				distanceTarget = -5.0;
				controller.setRelativeTarget(angleTarget, distanceTarget);
			} else if(controller.wallOnBothSides()) {
				strategy = "I'm in a corner.";
				angleTarget = -0.3 * Math.PI/2.0;
				distanceTarget = -5.0;
				stuckTimeMarker = elapsedTime;
				controller.setRelativeTarget(angleTarget, distanceTarget);
			} else if(controller.wallOnLeft()){
				strategy = "There's a wall to the left.";
				angleTarget = -0.3 * Math.PI/2.0;
				distanceTarget = -2.0;
				controller.setRelativeTarget(angleTarget, distanceTarget);
			} else if(controller.wallOnRight()){
				strategy = "There's a wall to the right.";
				angleTarget = 0.3 * Math.PI/2.0;
				distanceTarget = -2.0;
				controller.setRelativeTarget(angleTarget, distanceTarget);
			} else if(!controller.closeToWall()) {
				goToBalls();
			}
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
			System.out.println("Waiting for the game to start...");
			while(!client.gameStarted());
			System.out.println("The game has started!");
		}
		//mouth.speak("The game has started!");
		
		this.world = new RobotWorld(null);//client);
		this.controller.setCurrentAngle(world.getMap().startPose.theta);
		this.angleTarget = 0;//world.getMap().startPose.theta;
		this.startLooking();
		controller.setUpComm();
		 
        angleTarget = 0;
        distanceTarget = 8.0;
        controller.setRelativeTarget(angleTarget, distanceTarget);

	}
	
	void loop() {
		this.elapsedTime = time() - startTime;
		++counter;
		updateWorld();
		speak();
		sendStrategy();
		controller.updateMotionData();
		
		if((USE_BOTCLIENT && !client.gameStarted()) || elapsedTime > (1000 * 60 * 3) + 1000) {
			System.out.println("Game over!");
			strategy = "Game over!";
			speak();
			stopLooking();
		}

		pingPong();
		
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
			System.out.printf("\tDistance Target:\t%.2f\n", distanceTarget);
			System.out.printf("\tAngle Target:\t%.2f\n", angleTarget);
			System.err.println("Strategy:\t" + strategy);
		}
	}
	
}