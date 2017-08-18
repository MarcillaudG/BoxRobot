package fr.irit.smac.boxrobots;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import fr.irit.smac.amak.Agent;
import fr.irit.smac.amak.Amas;
import fr.irit.smac.amak.Scheduling;
import fr.irit.smac.boxrobots.Robot.Behavior;

public class Storehouse extends Amas<World>{

	public static final int NB_ROBOT = 50;
	
	private int nbBox;
	
	private int currentNbBox;

	public static final int NB_BOX = 1;
	
	private List<Boxs> boxs = new ArrayList<Boxs>();
	
	public Storehouse(World environment) {
		super(environment, Scheduling.UI);
		nbBox = 0;
		currentNbBox = 0;
		Random r = new Random();
		for(int i = 0; i<World.HEIGHT; i++){
			for(int j = 0; j < World.WIDTH; j++){
				Area ar = this.getEnvironment().getAreas()[i][j];
				if(ar.getIsBox()){
					boxs.add(new Boxs(ar.getX(),ar.getY()));
				}
			}
			/*int dx = r.nextInt(10);
			int dy = r.nextInt(40)+10;
			boxs.add(new Boxs(dx,dy));*/
		}
	}
	

	/**
	 * Return the list of Boxs
	 * 
	 * @return boxs
	 */
	public List<Boxs> getBoxs() {
		return boxs;
	}


	@Override
	protected void onInitialAgentsCreation() {
		for (int i = 0; i < Storehouse.NB_ROBOT;i++){
			new Robot(this,Behavior.CARLO);
		}
	}


	/**
	 * Return all the agents in an area
	 * 
	 * @param areaByPosition
	 * 
	 * @return all the agents
	 */
	public Robot[] getAgentsInArea(Area areaByPosition) {
		List<Robot> res = new ArrayList<>();
		for (Agent<?, World> agent : agents) {
			if (((Robot) agent).getCurrentArea() == areaByPosition)
				res.add((Robot) agent);
		}
		return res.toArray(new Robot[0]);
	}
	
	/**
	 * Return if a robot is in an area
	 * @param areaByPosition
	 * @return true if there is one
	 */
	public boolean isRobotInArea(Area areaByPosition){
		boolean ret = false;
		for (Agent<?, World> agent : agents) {
			if (((Robot) agent).getCurrentArea() == areaByPosition)
				ret = true;
		}
		return ret;
	}

	/**
	 * No longer used
	 */
	public void releaseBox() {
		Random r = new Random();
		currentNbBox++;
		this.nbBox++;
		boolean areaFree = false;
		int r1 = 0;
		int r2 = 0;
		while(!areaFree){
			r1 = r.nextInt(World.CLAIM[1]-World.CLAIM[0])+World.CLAIM[0];
			r2 = r.nextInt(World.CLAIM[3]-World.CLAIM[2])+World.CLAIM[2];
			Area ar = this.getEnvironment().getAreaByPosition(r1, r2);
			if(!ar.getIsBox() && !ar.getIsWall()){
				areaFree = true;
			}
		}
		//this.boxs.add(new Boxs(r1,r2));
		this.getEnvironment().getAreaByPosition(r1, r2).setBox(true);
	}
	

	/**
	 * No longer used
	 * @param dx
	 * @param dy
	 */
	public void pickingBox(int dx, int dy) {
		/*int toRemove = -1;
		for(int i = 0; i < boxs.size(); i++){
			Boxs box = boxs.get(i);
			if(box.getDx() == dx && box.getDy() == dy){
				toRemove = i;
			}
		}*/
		Area ar = this.getEnvironment().getAreaByPosition(dx, dy);
		if(ar.getIsBox())
			this.getEnvironment().getAreaByPosition(dx, dy).setBox(false);
		//boxs.remove(toRemove);
		this.currentNbBox--;
		
	}
	
	/**
	 * Return the number of box currently in the storehouse
	 * 
	 * @return currentNbBox
	 */
	public int getCurrentNbBox(){
		return this.currentNbBox;
	}
	
	/**
	 * Set the behavior of all the robots
	 * 
	 * @param behavior
	 */
	public void setBehavior(Behavior behavior){
		for(Agent<?,World> agent : agents){
			((Robot) agent).setBehavior(behavior);
		}
	}
	
	/**
	 * Return if the input area is in the claim zone
	 * @param ar
	 * 			The area
	 * @return true if the area is in
	 */
	public boolean isInClaimZone(Area ar){
		if(ar.getX() >= World.CLAIM[0] && ar.getX() <= World.CLAIM[1] && ar.getY() >= World.CLAIM[2] && ar.getY() <= World.CLAIM[3])
			return true;
		return false;
	}
	

	/**
	 * Return if the input area is in the release zone
	 * @param ar
	 * 			The area
	 * @return true if the area is in
	 */
	public boolean isInReleaseZone(Area ar){
		if(ar.getX() >= World.RELEASE[0] && ar.getX() <= World.RELEASE[1] && ar.getY() >= World.RELEASE[2] && ar.getY() <= World.RELEASE[3])
			return true;
		return false;
	}

}
