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
	private RobotComm comm;
	
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
	 * Create a new brain with a given eye and comm.
	 * 
	 * @param eye
	 * @param comm
	 */
	public RobotBrain(RobotEye eye, RobotComm comm, long startTime) {
		this.eye = eye;
		this.comm = comm;
		this.positionTarget = new Point(0, 0);
		this.angleTarget = 0;
		this.startTime = startTime;
		this.counter = 0;
	}

	public void loop() {
		this.elapsedTime = System.currentTimeMillis() - startTime;
		++counter;
		
		if(elapsedTime < 1000 * 60 * 1) {
			List<List<Integer>> lines = new ArrayList<List<Integer>>();
			List<List<Float>> redCircles = new ArrayList<List<Float>>(),
					greenCircles = new ArrayList<List<Float>>();

			eye.process(lines, redCircles, greenCircles);
			RobotWorld world = new RobotWorld(lines, redCircles, greenCircles);

			RobotWorld.Ball largestBall = world.getLargestBall();
			if(largestBall != null) {
				angleTarget = pixelToAngle(largestBall.getX());
			}
			//TODO: set angle: angleTarget.
		} else if (elapsedTime < 1000 * 60 * 3) {
			//TODO: Figure out what to do
		}
		else {
			System.out.println("Game over.");
		}
		
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