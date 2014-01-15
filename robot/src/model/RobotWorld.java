package model;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Keeps track of the objects in the world.
 * @author vmayar
 *
 */
public class RobotWorld {
	
	/** List of walls. */
	private List<Wall> walls;
	
	/** List of balls sorted by radius, which represents proximity to the robot. */
	private List<Ball> balls;
	//TODO: reactors, energy silo.
	
	/**
	 * Create a new instance of robot world from detected lines
	 * and circles.  Uses the circles to form balls and walls.
	 * 
	 * @param lines
	 * @param circles
	 */
	@SuppressWarnings("unchecked")
	public RobotWorld(List<List<Integer>> lines, List<List<Float>> redCircles,
			List<List<Float>> greenCircles) {
		balls = new ArrayList<Ball>();
		for(List<Float> ball : redCircles) {
			balls.add(new Ball(ball.get(0), ball.get(1), ball.get(2), "red"));
		}
		for(List<Float> ball : greenCircles) {
			balls.add(new Ball(ball.get(0), ball.get(1), ball.get(2), "green"));
		}
		Collections.sort(balls);
		//TODO: create walls.
	}
	
	public List<Ball> getBalls() {
		return new ArrayList<Ball>(balls);
	}
	
	public Ball getLargestBall() {
		if(!balls.isEmpty()) return balls.get(0);
		return null;
	}
	
	public List<Ball> getRedBalls() {
		List<Ball> redBalls = new ArrayList<Ball>();
		for(Ball ball : balls) {
			if(ball.getColor() == Ball.Color.RED) redBalls.add(ball);
		}
		return redBalls;
	}
	
	public List<Ball> getGreenBalls() {
		List<Ball> redBalls = new ArrayList<Ball>();
		for(Ball ball : balls) {
			if(ball.getColor() == Ball.Color.GREEN) redBalls.add(ball);
		}
		return redBalls;
	}
	
	/**
	 * Represents a wall in the robot's head.
	 * @author vmayar
	 *
	 */
	public static class Wall {
		//TODO:implement
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
			return this.getRadius() - ((Ball)other).getRadius() > 0 ? 1 : -1;
		}
	}

}
