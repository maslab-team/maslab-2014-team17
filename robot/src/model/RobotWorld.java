package model;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import BotClient.BotClient;
import robot.RobotEye;
import comm.BotClientMap;

/**
 * Keeps track of the objects in the world.
 * @author vmayar
 *
 */
public class RobotWorld {
	
	private BotClientMap map;
	
	/** Lists of balls sorted by radius, which represents proximity to the robot. */
	private List<Ball> greenBalls, redBalls;
	//TODO: reactors, energy silo.
	
	/**
	 * Create a new instance of robot world, initially empty.
	 */
	public RobotWorld() {
		this.greenBalls = new ArrayList<Ball>();
		this.redBalls = new ArrayList<Ball>();
		this.map = BotClientMap.getDefaultMap();
	}
	
	public RobotWorld(BotClient client) {
		if(client == null) {
			this.greenBalls = new ArrayList<Ball>();
			this.redBalls = new ArrayList<Ball>();
			this.map = BotClientMap.getDefaultMap();
		} else {
			this.greenBalls = new ArrayList<Ball>();
			this.redBalls = new ArrayList<Ball>();
			this.map = new BotClientMap();
			String mapString = client.getMap();
			System.out.println("Received map: " + mapString);
			if(mapString != null) {
				this.map.load(client.getMap());
				System.out.println("Map: " + map);
			} else {
				this.map = BotClientMap.getDefaultMap();
			}
		}
	}
	
	/**
	 * Return the underlying map of the world.
	 * 
	 * @return
	 */
	public BotClientMap getMap() {
		return map;
	}
	
	/**
	 * Create a new instance of robot world from detected
	 * circles.  Uses the circles to form balls.
	 * 
	 * @param lines
	 * @param circles
	 */
	public RobotWorld(List<List<Integer>> lines, List<List<Float>> redCircles,
			List<List<Float>> greenCircles) {
		this();
		this.update(lines, redCircles, greenCircles);
	}
	
	/**
	 * Updates the world given the circles seen by the eye.
	 * 
	 * @param redCircles
	 * @param greenCircles
	 */
	@SuppressWarnings("unchecked")
	public void update(List<List<Integer>> lines, List<List<Float>> redCircles,
			List<List<Float>> greenCircles) {
		redBalls.clear();
		greenBalls.clear();
		for(List<Float> ball : redCircles) {
			redBalls.add(new Ball(ball.get(0), ball.get(1), ball.get(2), "red"));
		}
		for(List<Float> ball : greenCircles) {
			greenBalls.add(new Ball(ball.get(0), ball.get(1), ball.get(2), "green"));
		}
		Collections.sort(redBalls);
		Collections.sort(greenBalls);
		//TODO: create walls.
	}
	
	/**
	 * Updates the world given circles seen by the eye
	 * as an instance of RobotEye.Data.
	 * 
	 * @param redCircles
	 * @param greenCircles
	 */
	@SuppressWarnings("unchecked")
	public void update(RobotEye.Data data) {
		redBalls.clear();
		greenBalls.clear();
		for(List<Float> ball : data.getRedCircles()) {
			redBalls.add(new Ball(ball.get(0), ball.get(1), ball.get(2), "red"));
		}
		for(List<Float> ball : data.getGreenCircles()) {
			greenBalls.add(new Ball(ball.get(0), ball.get(1), ball.get(2), "green"));
		}
		Collections.sort(redBalls);
		Collections.sort(greenBalls);
		//TODO: create walls.
	}
	
	public List<Ball> getBalls() {
		List<Ball> ret = new ArrayList<Ball>(redBalls);
		ret.addAll(greenBalls);
		return ret;
	}
	
	public Ball getLargestBall() {
		Ball largestRedBall = getLargestRedBall();
		Ball largestGreenBall = getLargestGreenBall();
		if(largestRedBall == null) return largestGreenBall;
		else if(largestGreenBall == null) return largestRedBall;
		else if(largestRedBall.compareTo(largestGreenBall) > 0) {
			return largestRedBall;
		}
		return largestGreenBall;
	}
	
	public Ball getLargestRedBall() {
		if(!redBalls.isEmpty()) return redBalls.get(0);		
		return null;
	}
	
	public Ball getLargestGreenBall() {
		if(!greenBalls.isEmpty()) return greenBalls.get(0);		
		return null;
	}
	
	public List<Ball> getRedBalls() {
		return new ArrayList<Ball>(redBalls);
	}
	
	public List<Ball> getGreenBalls() {
		return new ArrayList<Ball>(greenBalls);

	}
	
	/**
	 * Represents a ball in the robot's head.
	 * @author vmayar
	 *
	 */
	public static class Ball implements Comparable {
		private float radius;
		private int x;
		private int y;
		private Color color;
		
		public Ball(float x, float y, float radius, String color) {
			this.radius = radius;
			this.x = (int)x;
			this.y = (int)y;
			this.color = Color.fromString(color);
		}
		
		public enum Color {
			RED, GREEN;
			static Color fromString(String str) {
				if (str == null) return null;
				if (str.equalsIgnoreCase("red")) return Color.RED;
				else if (str.equalsIgnoreCase("green")) return Color.GREEN;
				else return null;
			}
		}
		
		public float getRadius() {
			return radius;
		}
		public int getX() {
			return x;
		}
		public int getY() {
			return y;
		}
		public Color getColor() {
			return color;
		}
		public int compareTo(Object other) {
			if (other == null) return -1;
			if (!(other instanceof Ball)) return -1;
			//return this.getRadius() - ((Ball)other).getRadius() > 0 ? -1 : 1;
			return (this.getY() - ((Ball)other).getY() > 0) ? -1 : 1;
		}
	}

}
