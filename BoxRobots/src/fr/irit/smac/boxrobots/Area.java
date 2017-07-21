package fr.irit.smac.boxrobots;

import java.util.Random;

/**
 * An area represents a small part of the world that can be scanned by a drone
 * in one cycle
 *
 */
public class Area {

	/**
	 * The amount of time (in cycles) since when the area hasn't been scanned
	 */
	private double timeSinceLastSeen = 1000;
	/**
	 * The X coordinate of the area
	 */
	private int x;
	/**
	 * The Y coordinate of the area
	 */
	private int y;

	private boolean isWall;

	private boolean isBox;

	/**
	 * Constructor of the area
	 * 
	 * @param x
	 *            X coordinate
	 * @param y
	 *            Y coordinate
	 */
	public Area(int x, int y) {
		// Set the position
		this.x = x;
		this.y = y;
		Random r = new Random();
		if(x >= 30 && x < 50 && y >= 1 && y <= 58){
			isWall = true;
			isBox = false;
		}
		else{
			isWall = false;
			if(x < World.CLAIM[1] && y > World.CLAIM[2] && y < World.CLAIM[3]){
				if(r.nextInt(4) == 0)
					isBox = true;
				else
					isBox = false;
			}
			else{
				isBox = false;
			}
		}

	}

	/**
	 * Getter for the X coordinate
	 * 
	 * @return the x coordinate
	 */
	public int getX() {
		return x;
	}

	/**
	 * Getter for the Y coordinate
	 * 
	 * @return the y coordinate
	 */
	public int getY() {
		return y;
	}

	public boolean getIsWall(){
		return isWall;
	}

	/**
	 * This method is called when the drone scans the area
	 * 
	 * @param robot
	 *            The robot which scans the area
	 */
	public void seen(Robot robot) {
		timeSinceLastSeen = 0;
	}

	/**
	 * Getter for the amount of time since last scan
	 * 
	 * @return the amount of time since last scan
	 */
	public double getTimeSinceLastSeen() {
		return timeSinceLastSeen;
	}

	/**
	 * Update the time since last scan at each cycle
	 */
	public void cycle() {
		timeSinceLastSeen++;
	}

	/**
	 * Manually set a hgh criticality to request a scan on a specific area
	 */
	public void setCritical() {
		timeSinceLastSeen = 1000;
	}

	/**
	 *  Change the area in a wall
	 *  
	 * @param isWall
	 */
	public void setIsWall(boolean isWall){
		this.isWall = isWall;
		this.isBox = false;
	}

	/**
	 * Change the area in a box
	 * 
	 * @param isBox
	 */
	public void setBox(boolean isBox){
		this.isBox = isBox;
		this.isWall = false;
	}

	/**
	 * Return if the area is a box or not
	 * 
	 * @return  isBox
	 */
	public boolean getIsBox(){
		return this.isBox;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + x;
		result = prime * result + y;
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Area other = (Area) obj;
		if (x != other.x)
			return false;
		if (y != other.y)
			return false;
		return true;
	}
}
