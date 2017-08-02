package fr.irit.smac.boxrobots;


import java.util.Random;

import fr.irit.smac.amak.Environment;

public class World extends Environment{
	/**
	 * Areas in the world
	 */
	private Area[][] areas;
	/**
	 * Number of areas in width
	 */
	public final static int WIDTH = 80;
	/**
	 * Number of areas in height
	 */
	public final static int HEIGHT = 60;

	/**
	 * The claim zone
	 */
	public static int[] CLAIM = {0,10,10,50}; 

	/**
	 * The release zone
	 */
	public static int[] RELEASE = {70,80,10,50}; 

	/**
	 * Create the various areas
	 */
	@Override
	public void onInitialization() {
		areas = new Area[HEIGHT][WIDTH];
		for (int x = 0; x < WIDTH; x++) {
			for (int y = 0; y < HEIGHT; y++) {
				areas[y][x] = new Area(x, y);
			}
		}
	}

	/**
	 * Inform each area at the end of each cycle
	 */
	@Override
	public void onCycleEnd() {
		for (int x = 0; x < WIDTH; x++) {
			for (int y = 0; y < HEIGHT; y++) {
				areas[y][x].cycle();
			}
		}
	}

	/**
	 * Getter for the areas
	 * 
	 * @return the areas array
	 */
	public Area[][] getAreas() {
		return areas;
	}

	/**
	 * Get an area at a specific coordinate
	 * 
	 * @param dx
	 *            the x coordinate
	 * @param dy
	 *            the y coordinate
	 * @return the area
	 */
	public Area getAreaByPosition(int dx, int dy) {

		if (dx < 0 || dy < 0 || dx >= WIDTH || dy >= HEIGHT)
			return null;
		return areas[dy][dx];
	}

	public void randomMaze(int nbPath){
		for(int x = 30; x < 50;x++){
			for(int y = 0; y < 60; y++ ){
				this.getAreaByPosition(x, y).setIsWall(true);
			}
		}
		for(int i = 0 ; i < nbPath ; i++){
			Random r = new Random();
			int start = 30;
			int currentx = start;
			int currenty = r.nextInt(60);
			while(currentx != 50){
				int dir = r.nextInt(3);
				switch(dir){
				case 0:
					if(currenty >0){
						currenty--;
					}
					break;
				case 1:
					currentx++;
					break;
				case 2:
					if(currenty <59){
						currenty++;
					}
					break;
				/*case 2:
					if(currentx > 30){
						currentx--;
					}
					break;*/
				}
				this.getAreaByPosition(currentx,currenty).setIsWall(false);
			}
		}
	}
	
	public void randomPath(int nbPath){
		for(int x = 30; x < 50;x++){
			for(int y = 0; y < 60; y++ ){
				this.getAreaByPosition(x, y).setIsWall(true);
			}
		}
		for(int i = 0 ; i < nbPath ; i++){
			Random r = new Random();
			int start = 30;
			int currentx = start;
			int currenty = r.nextInt(60);
			this.getAreaByPosition(currentx,currenty).setIsWall(false);
			while(currentx != 50){
				currentx++;
				this.getAreaByPosition(currentx,currenty).setIsWall(false);
			}
		}
	}

}
