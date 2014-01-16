package tablet;

import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

import model.RobotWorld;

/**
 * Contains method necessary for processing inputs
 * and making decisions.
 * 
 * @author vmayar
 *
 */
public class RobotBrain {
	
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
	
	/** Represents relative position target in xy-plane. */
	private Point positionTarget;
	
	/**
	 * Represents relative angular position target in radians.
	 * Counterclockwise direction is positive.
	 */
	private double angleTarget;
	
	/**
	 * Start time of the game, in millis since epoch.
	 */
	private long startTime;
	
	/**
	 * Time elapsed since game start.
	 */
	private long elapsedTime;

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
		this.world = world;
		this.positionTarget = new Point(0, 0);
		this.angleTarget = 0;
		this.startTime = startTime;
		this.counter = 0;
		this.eyeData = new RobotEye.Data();
		this.startLooking();
	}
	
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

	void loop() {
		this.elapsedTime = System.currentTimeMillis() - startTime;
		++counter;
		updateWorld();
		
		if(elapsedTime < 1000 * 60 * 1) {
			
			RobotWorld.Ball largestBall = world.getLargestBall();
			if(largestBall != null) {
				angleTarget = pixelToAngle(largestBall.getX());
			}
			//TODO: set angle: angleTarget.
			
		} else if (elapsedTime < 1000 * 60 * 3) {
			//TODO: Figure out what to do
		}
		else {
			stopLooking();
			System.out.println("Game over.");
		}
		
		controller.sendControl();
		debug();
	}
	
	/**
	 * Given a horizontal pixel coordinate, return the angle from
	 * the direction of the camera to the given coordinate.
	 * 
	 * @param pixelX
	 * @return
	 */
	private double pixelToAngle(int pixelX) {
		int distFromCenter = pixelX - RobotEye.IMAGE_WIDTH/2;
		return Math.atan2(distFromCenter, RobotEye.IMAGE_DEPTH_IN_PIXELS);
		
	}
	
	private void debug() {
		System.out.printf("ROBOT INFO\n");
		System.out.printf("\tTime: %d\n", elapsedTime);
		System.out.printf("\tCounter: %d\n", counter);
		System.out.printf("\tPosition Target: " + positionTarget + "\n");
		System.out.printf("\tAngle Target: %.2f\n", angleTarget);
	}

}