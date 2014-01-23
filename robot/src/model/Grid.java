package model;

import comm.BotClientMap;

public class Grid {
	private static final int GRID_SIZE_X = 100;
	private static final int GRID_SIZE_Y = 100;
	
	private Square[][] grid;
	
	Grid(BotClientMap map) {
		grid = new Square[GRID_SIZE_Y][GRID_SIZE_X];
	}
	
	static class Square {
		int x, y;
		boolean occupied;
		
		Square (int x, int y, boolean occupied) {
			this.x = x;
			this.y = y;
			this.occupied = occupied;
		}
	}
}
