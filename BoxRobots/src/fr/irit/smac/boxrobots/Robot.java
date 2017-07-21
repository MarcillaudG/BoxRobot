package fr.irit.smac.boxrobots;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Random;

import fr.irit.smac.amak.Agent;

public class Robot extends Agent<Storehouse, World>{

	private static final int VIEW_RADIUS = 5;

	public static int MEMORY_SIZE = 20;

	private static final int CRITIC = 10;

	private int dx;

	private int dy;

	private boolean isCarrying;

	private boolean isReleasing;

	private boolean isPicking;

	private boolean isReturning;

	private Area closestBox;

	private Direction currentDirection;

	private Map<Area,Integer> memory;

	private Queue<Area> carryingQueue;

	private Queue<Area> claimQueue;

	public enum Behavior{
		TOWARD,RANDOM,MEMORY,COOP
	}

	public enum Module{
		DIRECT,MEMORY,COOP
	}

	public enum Direction{
		UP,RIGHT,DOWN,LEFT
	}

	/**
	 * The areas perceived by the agent during the perception phase
	 */
	private Area[][] view = new Area[VIEW_RADIUS * 2 + 1][VIEW_RADIUS * 2 + 1];
	/**
	 * The drone perceived by the agent during the perception phase
	 */
	private Robot[][][] neighborsView = new Robot[VIEW_RADIUS * 2 + 1][VIEW_RADIUS * 2 + 1][];

	/**
	 * The four area the robot can reach with one movement
	 */
	private Area[] targetsPossible = new Area[4];



	/**
	 * The area the drone will try to reach during the action phase
	 */
	private Area targetArea;

	/**
	 * The current area of the drone. Located at dx, dy
	 */
	private Area currentArea;

	private Behavior behavior;

	/**
	 * The list of all the modules the robot will use
	 */
	private List<Module> modules;

	public Robot(Storehouse amas,Behavior behavior) {
		super(amas);
		this.behavior = behavior;
		this.modules = new ArrayList<Module>();
		this.memory = new HashMap<Area,Integer>(); 
		this.carryingQueue = new LinkedList<Area>();
		this.claimQueue = new LinkedList<Area>();
		Random r = new Random();
		if(r.nextInt(2)%2==0){
			this.dx = r.nextInt(29);
			this.dy = r.nextInt(60);
		}

		else{
			this.dx = r.nextInt(29)+51;
			this.dy = r.nextInt(60);
		}
		this.isCarrying = false;
		this.isReleasing = false;
		this.isReturning = false;
		this.isPicking = false;
	}


	public int getX(){
		return dx;
	}

	public int getY(){
		return dy;
	}

	/**
	 * Initialize the first area of the robot
	 */
	@Override
	protected void onReady() {
		currentArea = amas.getEnvironment().getAreaByPosition(dx, dy);
	}

	@Override
	protected void onPerceive() {
		// Clear the last set neighbors list
		clearNeighbors();

		//Perceive the four areas the robot can reach
		targetsPossible[0] = amas.getEnvironment().getAreaByPosition(dx, dy-1);
		targetsPossible[1] = amas.getEnvironment().getAreaByPosition(dx+1, dy);
		targetsPossible[2] = amas.getEnvironment().getAreaByPosition(dx, dy+1);
		targetsPossible[3] = amas.getEnvironment().getAreaByPosition(dx-1, dy);

		// Check areas in a range of 5x5
		for (int x = -VIEW_RADIUS; x <= VIEW_RADIUS; x++) {
			for (int y = -VIEW_RADIUS; y <= VIEW_RADIUS; y++) {
				Area areaByPosition = amas.getEnvironment().getAreaByPosition(dx + x, dy + y);
				//Get the closest box
				if(areaByPosition !=null && areaByPosition.getIsBox()) {
					if(closestBox == null)
						closestBox = areaByPosition;
					else{
						double distanceToClosest = Math.sqrt(Math.pow(dx-closestBox.getX(), 2)+Math.pow(dy-closestBox.getY(), 2));
						double distanceToArea = Math.sqrt(Math.pow(dx-areaByPosition.getX(), 2)+Math.pow(dy-areaByPosition.getY(), 2));
						if(distanceToClosest > distanceToArea)
							closestBox = areaByPosition;
					}
				}
				// store the seen areas
				view[y + VIEW_RADIUS][x + VIEW_RADIUS] = areaByPosition;
				Robot[] agentsInArea = amas.getAgentsInArea(areaByPosition);
				// store the seen agents
				neighborsView[y + VIEW_RADIUS][x + VIEW_RADIUS] = agentsInArea;
				// Set seen agents as neighbors
				addNeighbor(agentsInArea);
			}
		}
	}

	@Override
	protected void onDecide() {
		switch(behavior){
		case TOWARD:
			toward();
			break;
		case RANDOM:
			random();
		default:
			break;
		}
	}

	/**
	 * If the robot can't go to its objective it will use a random pattern to find a way
	 */
	private void random() {
		boolean hasATarget = false;
		// If the robot is carrying a box
		if(isCarrying){
			//If it is in the release Area 
			if(amas.isInReleaseZone(this.currentArea))
				isReleasing = true;
			// Else it try to go toward it
			else{
				if(this.modules.contains(Module.COOP)){
					checkGoBack(1);
					if(this.isReturning){
						if(checkFreedom(this.targetsPossible[3])){
							this.targetArea = this.targetsPossible[3];
						}
					}
				}
				if(!this.isReturning){
					//TODO
					if(this.dx < World.RELEASE[0]){
						if(this.modules.contains(Module.MEMORY)){
							hasATarget = useYourBrain(Direction.RIGHT);
						}
						if(!hasATarget){
							Area ar = targetsPossible[1];
							boolean free = checkFreedom(ar);

							if(free){
								this.targetArea = ar;
								this.currentDirection = null;
							}
							else{
								goRandom();
							}
						}
					}
					else{
						if(this.dy < World.RELEASE[2]){
							if(this.modules.contains(Module.MEMORY)){
								hasATarget = useYourBrain(Direction.DOWN);
							}
							if(!hasATarget){
								goDown();
								this.currentDirection = null;
							}
						}
						else{
							if(this.modules.contains(Module.MEMORY)){
								hasATarget = useYourBrain(Direction.UP);
							}
							if(!hasATarget){
								goUp();
								this.currentDirection = null;
							}
						}
					}
				}
			}
		}
		else{
			// If the robot is in the claim area
			if(amas.isInClaimZone(this.currentArea)){
				for (int x = -1; x <= 1; x++) {
					for (int y = -1; y <= 1; y++) {
						Area ar = amas.getEnvironment().getAreaByPosition(dx + x, dy + y);
						if(ar != null && ar.getIsBox()){
							isPicking = true;
							this.targetArea = ar;
							this.currentDirection = null;
						}
					}
				}
				if(!isPicking){
					if(closestBox != null){
						this.targetArea = closestBox;
						this.currentDirection = null;
					}
					else{
						goRandomInClaim();
						this.currentDirection = null;
					}
				}
			}
			else{
				if(this.modules.contains(Module.COOP)){
					checkGoBack(3);
					if(this.isReturning){
						if(checkFreedom(this.targetsPossible[1])){
							this.targetArea = this.targetsPossible[1];
						}
					}
				}
				if(!this.isReturning){
					if(this.dx > World.CLAIM[1]){
						if(this.modules.contains(Module.MEMORY)){
							hasATarget = useYourBrain(Direction.LEFT);
						}
						if(!hasATarget){
							Area ar = targetsPossible[3];
							boolean free = checkFreedom(ar);
							if(free){
								this.targetArea = ar;
								this.currentDirection = null;
							}
							else{
								goRandom();
							}
						}
					}
					else{
						if(this.dy < World.CLAIM[2]){
							if(this.modules.contains(Module.MEMORY)){
								hasATarget = useYourBrain(Direction.DOWN);
							}
							if(!hasATarget){
								goDown();
								this.currentDirection = null;
							}
						}
						else{
							if(this.modules.contains(Module.MEMORY)){
								hasATarget = useYourBrain(Direction.UP);
							}
							if(!hasATarget){
								goUp();
								this.currentDirection = null;
							}
						}
					}
				}
			}
		}

	}

	//TODO
	private boolean useYourBrain(Direction d) {
		boolean hasATarget = false;
		/*if(this.currentDirection != null){
			d = this.currentDirection;
		}*/
		Queue<Area> tmpQueue = null;
		if(this.isCarrying)
			tmpQueue = this.carryingQueue;
		else
			tmpQueue = this.claimQueue;
		Double[] prob = {0.0,0.0,0.0,0.0};

		// We define the first probabilities 
		switch(d){
		case RIGHT:
			prob[1] = 100.0;
			prob[3] = 10.0;
			prob[0] = prob[2] = 20.0;
			break;
		case LEFT:
			prob[3] = 100.0;
			prob[1] = 10.0;
			prob[0] = prob[2] = 20.0;
			break;
		case UP:
			prob[0] = 100.0;
			prob[2] = 10.0;
			prob[1] = prob[3] = 20.0;
			break;
		case DOWN:
			prob[2] = 100.0;
			prob[0] = 10.0;
			prob[1] = prob[3] = 20.0;
			break;
		}

		for(int i = 0 ; i < 4 ; i++){
			if(!this.checkFreedom(this.targetsPossible[i])){
				prob[i] = -1.0;
			}
		}
		switch(d){
		case RIGHT:
			if(prob[1] == -1.0 && this.currentDirection != null && this.currentDirection != d)
				return this.useYourBrain(this.currentDirection);
			break;
		case LEFT:
			if(prob[3] == -1.0 && this.currentDirection != null && this.currentDirection != d)
				return this.useYourBrain(this.currentDirection);
			break;
		case UP:
			if(prob[0] == -1.0 && this.currentDirection != null && this.currentDirection != d)
				return this.useYourBrain(this.currentDirection);
			break;
		case DOWN:
			if(prob[2] == -1.0 && this.currentDirection != null && this.currentDirection != d)
				return this.useYourBrain(this.currentDirection);
			break;
		}


		// Now we look for all critics zone
		//The zone above
		for(int y = 0; y < VIEW_RADIUS; y++){
			for(int x = y; x <= 2*VIEW_RADIUS-y; x++){
				if(tmpQueue.contains(this.view[y][x])){
					if(y != VIEW_RADIUS && x != VIEW_RADIUS)
						prob[0] -= (10/(Math.abs(VIEW_RADIUS-y)+Math.abs(VIEW_RADIUS-x)));
				}
			}
		}
		//The zone to the right
		for(int x = VIEW_RADIUS*2; x > VIEW_RADIUS; x--){
			for(int y = x; y >= x - VIEW_RADIUS ; y--){
				if(tmpQueue.contains(this.view[y][x])){
					if(y != VIEW_RADIUS && x != VIEW_RADIUS)
						prob[1] -= (10/(Math.abs(VIEW_RADIUS-y)+Math.abs(VIEW_RADIUS-x)));
				}
			}
		}
		//The zone to the bottom
		for(int y = 2*VIEW_RADIUS; y > VIEW_RADIUS; y--){
			for(int x = y; x >= y - VIEW_RADIUS; x--){
				if(tmpQueue.contains(this.view[y][x])){
					if(y != VIEW_RADIUS && x != VIEW_RADIUS)
						prob[2] -= (10/(Math.abs(VIEW_RADIUS-y)+Math.abs(VIEW_RADIUS-x)));
				}
			}
		}
		//The zone to the left
		for(int x = 0; x < VIEW_RADIUS; x++){
			for(int y = x; y <= 2*VIEW_RADIUS-x; y++){
				if(tmpQueue.contains(this.view[y][x])){
					if(y != VIEW_RADIUS && x != VIEW_RADIUS)
						prob[3] -= (10/(Math.abs(VIEW_RADIUS-y)+Math.abs(VIEW_RADIUS-x)));
				}
			}
		}

		switch(d){
		case RIGHT:
			if(prob[1]==100.0){
				if( checkFreedom(this.targetsPossible[1])){
					this.targetArea = this.targetsPossible[1];
					this.currentDirection = null;
					return true;
				}
			}
			break;
		case LEFT:
			if(prob[3]==100.0 && checkFreedom(this.targetsPossible[3])){
				this.targetArea = this.targetsPossible[3];
				this.currentDirection = null;
				return true;
			}
			break;
		case UP:
			if(prob[0]==100.0 && checkFreedom(this.targetsPossible[0])){
				this.targetArea = this.targetsPossible[0];
				this.currentDirection = null;
				return true;
			}
			break;
		case DOWN:
			if(prob[2]==100.0 && checkFreedom(this.targetsPossible[2])){
				this.targetArea = this.targetsPossible[2];
				this.currentDirection = null;
				return true;
			}
			break;
		}

		List<Double> borne = new ArrayList<Double>();
		double sumProb = 0.0;
		for(int i = 0; i < 4; i++){
			if(prob[i]>0.0){
				sumProb += prob[i];
				prob[i] = sumProb;
				borne.add(prob[i]);
			}
		}
		if(sumProb != 0){
			Random r = new Random();
			int result = r.nextInt((int) sumProb);
			int indBorne = 0;
			System.out.println("Sumprob : "+ sumProb + " Result : "+result);
			if(result >=0 && borne.size()>0){
				if(prob[0] > 0){
					if(result < borne.get(indBorne)){
						this.targetArea = this.targetsPossible[0];
						this.currentDirection = Direction.UP;
						return true;
					}
				}
				if(prob[1]>0){
					if(result >= borne.get(indBorne) && (borne.size() <= indBorne || result < borne.get(indBorne)+1) ){
						this.targetArea = this.targetsPossible[1];
						this.currentDirection = Direction.RIGHT;
						return true;
					}
					indBorne++;
				}
				if( prob[2]>0){ 
					if(result >= borne.get(indBorne) && (borne.size() <= indBorne || result < borne.get(indBorne)+1) ){
						this.targetArea = this.targetsPossible[2];
						this.currentDirection = Direction.DOWN;
						return true;
					}
					indBorne++;
				}
				if(prob[3]>0){
					if(result < borne.get(indBorne)){
						this.targetArea = this.targetsPossible[3];
						this.currentDirection = Direction.LEFT;
						return true;
					}
				}
			}
		}
		return false;
	}


	private void checkGoBack(int target) {
		Robot[] agentsInArea = amas.getAgentsInArea(this.targetsPossible[target]);
		switch(target%2){
		case 0:
			if(this.isCarrying){
				if(!isReturning){
					if(agentsInArea.length>0){
						if(!agentsInArea[0].isCarrying){
							if(this.distanceToRelease(this.currentArea) > this.distanceToClaim(this.targetsPossible[target])){
								this.isReturning = true;
							}
							else
								this.isReturning = false;
						}
						else{
							if(agentsInArea[0].isReturning)
								this.isReturning = true;
						}
					}
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[1]) && !checkFreedom(this.targetsPossible[3])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
					}
				}
			}
			else{
				if(!this.isReturning){
					if(agentsInArea.length>0){
						if(agentsInArea[0].isCarrying){
							if(this.distanceToClaim(this.currentArea) > this.distanceToRelease(this.targetsPossible[target]) ){
								this.isReturning = true;
							}
							else
								this.isReturning = false;
						}
						else{
							if(agentsInArea[0].isReturning)
								this.isReturning = true;
						}
					}
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[1]) && !checkFreedom(this.targetsPossible[3])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
					}
				}
			}
			break;
		case 1:
			if(this.isCarrying){
				if(agentsInArea.length>0){
					if(!agentsInArea[0].isCarrying){
						if(this.distanceToRelease(this.currentArea) > this.distanceToClaim(this.targetsPossible[target])){
							this.isReturning = true;
						}
						else
							this.isReturning = false;
					}
					else{
						if(agentsInArea[0].isReturning)
							this.isReturning = true;
					}
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[0]) && !checkFreedom(this.targetsPossible[2])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
						goRandom();
					}
				}
			}
			else{
				if(agentsInArea.length>0){
					if(agentsInArea[0].isCarrying){
						if(this.distanceToClaim(this.currentArea) > this.distanceToRelease(this.targetsPossible[target])){
							this.isReturning = true;
						}
						else
							this.isReturning = false;
					}
					else{
						if(agentsInArea[0].isReturning)
							this.isReturning = true;
					}
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[0]) && !checkFreedom(this.targetsPossible[2])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
						goRandom();
					}
				}
			}
			break;
			/*case 2:
			if(this.isCarrying){
				if(agentsInArea.length > 0 && !agentsInArea[0].isCarrying){ 
					if(this.distanceToRelease(this.currentArea) > this.distanceToClaim(this.targetsPossible[target])){
						this.isReturning = true;
					}
					else
						this.isReturning = false;
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[1]) && !checkFreedom(this.targetsPossible[3])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
					}
				}
			}
			else{
				if(agentsInArea.length > 0 && agentsInArea[0].isCarrying){ 
					if(this.distanceToRelease(this.currentArea) > this.distanceToClaim(this.targetsPossible[target])){
						this.isReturning = true;
					}
					else
						this.isReturning = false;
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[1]) && !checkFreedom(this.targetsPossible[3])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
					}
				}
			}
			break;
		case 3:
			if(this.isCarrying){
				if(agentsInArea.length > 0 && !agentsInArea[0].isCarrying){ 
					if(this.distanceToRelease(this.currentArea) > this.distanceToClaim(this.targetsPossible[target])){
						this.isReturning = true;
					}
					else
						this.isReturning = false;
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[0]) && !checkFreedom(this.targetsPossible[2])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
					}
				}
			}
			else{
				if(agentsInArea.length > 0 && agentsInArea[0].isCarrying){ 
					if(this.distanceToRelease(this.currentArea) > this.distanceToClaim(this.targetsPossible[target])){
						this.isReturning = true;
					}
					else
						this.isReturning = false;
				}
				if(this.isReturning){
					if(!checkFreedom(this.targetsPossible[0]) && !checkFreedom(this.targetsPossible[2])){
						this.isReturning = true;
					}
					else{
						this.isReturning = false;
					}
				}
			}
			break;*/
		}
	}


	/**
	 * If the robot has the toward behavior, it will do anything to go to its objective
	 */
	private void toward() {
		// If the robot is carrying a box
		if(isCarrying){
			//If it is in the release Area 
			if(amas.isInReleaseZone(this.currentArea))
				isReleasing = true;
			// Else it try to go toward it
			else{
				if(this.dx < World.RELEASE[0]){
					goRight();
				}
				else{
					if(this.dy < World.RELEASE[2]){
						goDown();
					}
					else
						goUp();
				}
			}
		}
		else{
			// If the robot is in the claim area
			if(amas.isInClaimZone(this.currentArea)){
				for (int x = -1; x <= 1; x++) {
					for (int y = -1; y <= 1; y++) {
						Area ar = amas.getEnvironment().getAreaByPosition(dx + x, dy + y);
						if(ar != null && ar.getIsBox()){
							isPicking = true;
							this.targetArea = ar;
						}
					}
				}
				if(!isPicking){
					if(closestBox != null){
						this.targetArea = closestBox;
					}
					else
						goRandomInClaim();
				}
			}
			else{
				if(this.dx > World.CLAIM[1])
					goLeft();
				else{
					if(this.dy < World.CLAIM[2]){
						goDown();
					}
					else
						goUp();
				}
			}
		}
	}


	/**
	 * The robot will move in random 
	 */
	private void goRandom() {
		ArrayList<Area> possible = new ArrayList<Area>();
		boolean moved = false;
		if(this.currentDirection != null){
			switch(this.currentDirection){
			case DOWN:
				if(checkFreedom(targetsPossible[2])){
					this.targetArea = targetsPossible[2];
					moved = true;
				}

				break;
			case UP:
				if(checkFreedom(targetsPossible[0])){
					this.targetArea = targetsPossible[0];
					moved = true;
				}
				break;
			case RIGHT:
				if(checkFreedom(targetsPossible[1])){
					this.targetArea = targetsPossible[1];
					moved = true;
				}
				break;
			case LEFT:
				if(checkFreedom(targetsPossible[3])){
					this.targetArea = targetsPossible[3];
					moved = true;
				}
				break;
			}
		}
		if(this.currentDirection == null || !moved || !this.modules.contains(Module.DIRECT)){
			for(int i = 0; i < 4; i++){
				if(checkFreedom(targetsPossible[i])){
					possible.add(targetsPossible[i]);
				}
			}
			Random r = new Random();
			if(possible.size()>0){
				this.targetArea = possible.get(r.nextInt(possible.size()));
				switch(this.currentArea.getY()-this.targetArea.getY()){
				case 1:
					this.currentDirection = Direction.UP;
					break;
				case -1:
					this.currentDirection = Direction.DOWN;
					break;
				default:
					//this.currentDirection = null;
					break;
				}
			}
			else
				this.currentDirection = null;
		}
		if(!this.modules.contains(Module.DIRECT))
			this.currentDirection = null;
	}


	/**
	 * The robot will move in random when it is in the claim zone 
	 * but it does not see any box
	 */
	private void goRandomInClaim() {
		ArrayList<Area> possible = new ArrayList<Area>();
		for(int i = 0; i < 4; i++){
			if(checkFreedom(targetsPossible[i])&&amas.isInClaimZone(targetsPossible[i])){
				possible.add(targetsPossible[i]);
			}
		}
		Random r = new Random();
		if(possible.size()>0)
			this.targetArea = possible.get(r.nextInt(possible.size()));
	}

	/**
	 * The robot try to go right
	 */
	private void goRight() {
		Area ar = targetsPossible[1];
		boolean free = checkFreedom(ar);
		if(free){
			this.targetArea = ar;
		}
		else{
			ar = targetsPossible[0];
			free = checkFreedom(ar);
			if(free){
				this.targetArea = ar;
			}
			else{
				ar = targetsPossible[2];
				free = checkFreedom(ar);
				if(free){
					this.targetArea = ar;
				}
			}
		}
	}

	/**
	 * The robot try to go left
	 */
	private void goLeft() {
		Area ar = targetsPossible[3];
		boolean free = checkFreedom(ar);
		if(free){
			this.targetArea = ar;
		}
		else{
			ar = targetsPossible[0];
			free = checkFreedom(ar);
			if(free){
				this.targetArea = ar;
			}
			else{
				ar = targetsPossible[2];
				free = checkFreedom(ar);
				if(free){
					this.targetArea = ar;
				}
			}
		}
	}

	/**
	 * The robot try to go up
	 */
	private void goUp() {
		Area ar = targetsPossible[0];
		boolean free = checkFreedom(ar);
		if(free){
			this.targetArea = ar;
		}
		else{
			goRandom();
		}
	}

	/**
	 * The robot try to go down
	 */
	private void goDown() {
		Area ar = targetsPossible[2];
		boolean free = checkFreedom(ar);
		if(free){
			this.targetArea = ar;
		}
		else{
			goRandom();
		}
	}

	/**
	 * Check if the given area has no obstacle
	 * 
	 * @param ar
	 * 			The area
	 * @return true if no obstacle 
	 */
	private boolean checkFreedom(Area ar){
		boolean free = true;
		if(ar != null && ar.getX() >=0 && ar.getY() >= 0 && ar.getX() <= 80 && ar.getY() <= 60){
			if(!ar.getIsWall()){
				if(amas.isRobotInArea(ar)){
					free = false;
				}
			}
			else{
				return false;
			}
		}
		else
			free = false;
		return free;

	}

	/**
	 * The robot release a box and become in the state of looking for one
	 */
	private void releaseBox() {
		this.isCarrying = false;
		this.isReleasing = false;
		this.amas.releaseBox();

	}


	@Override
	protected void onAct() {
		if(isReleasing){
			releaseBox();
		}
		else
			if(isPicking){
				pickingBox();
			}
			else
				if (targetArea != null) {
					closestBox = null;
					moveToward(targetArea);
					targetArea = null;
					if(this.isReturning && this.modules.contains(Module.MEMORY)){
						this.addToQueue(this.currentArea);
					}
				}
				else{
					if(this.modules.contains(Module.MEMORY)){
						this.addToQueue(this.currentArea);
					}
				}
		for(Area ar : this.memory.keySet()){
			this.memory.replace(ar, this.memory.get(ar)-1);
		}
	}


	private void pickingBox() {
		this.isPicking = false;
		this.isCarrying = true;
		Area ar = amas.getEnvironment().getAreaByPosition(targetArea.getX(), targetArea.getY());
		ar.setBox(false);
		this.amas.pickingBox(targetArea.getX(),targetArea.getY());
	}


	/**
	 * Clear the neighbors list
	 */
	private void clearNeighbors() {
		neighborhood.clear();
	}

	/**
	 * Getter for the area of the robot
	 * 
	 * @return the current area
	 */
	public Area getCurrentArea() {
		return currentArea;
	}

	/**
	 * Change the position of the robot to matches the area's
	 * 
	 * @param a
	 * 			The area
	 */
	protected void moveToward(Area a) {
		if (dx < a.getX())
			dx+= 1;
		else if (dx > a.getX())
			dx-= 1;
		if (dy < a.getY())
			dy+= 1;
		else if (dy > a.getY())
			dy-= 1;
		currentArea = amas.getEnvironment().getAreaByPosition(dx, dy);
		currentArea.seen(this);
	}

	/**
	 * Return if the robot is carrying or not
	 * 
	 * @return isCarrying
	 */
	public boolean isCarrying(){
		return this.isCarrying;
	}

	/**
	 * Changes the behavior
	 * 
	 * @param behaviour
	 */
	public void setBehavior(Behavior behavior){
		this.behavior = behavior;
	}

	/**
	 * Add a module for the robot
	 * 
	 * @param module
	 */
	public void addModule(Module module){
		this.modules.add(module);
	}

	/**
	 * Remove a module for the robot
	 * 
	 * @param module
	 */
	public void removeModule(Module module){
		this.modules.remove(module);
	}

	/**
	 * Return the states of the robot
	 * 
	 * @return isCarrying
	 */
	public boolean getCarrying(){
		return this.isCarrying;
	}


	/**
	 * Calculate the distance between an Area and the claim zone
	 * 
	 * @param ar
	 * 			The area
	 * @return ret
	 */
	private double distanceToClaim(Area ar){
		double ret = Double.MAX_VALUE;
		double xclaim = 0;
		double yclaim = 0;
		double xarea = new Double(ar.getX());
		double yarea = new Double(ar.getY());
		if(xarea >= World.CLAIM[0] && xarea <= World.CLAIM[1]){
			xclaim = xarea;
		}
		else{
			if(xarea <= World.CLAIM[0])
				xclaim = World.CLAIM[0];
			if(xarea >= World.CLAIM[1])
				xclaim = World.CLAIM[1];
		}


		if(yarea >= World.CLAIM[2] && yarea <= World.CLAIM[3]){
			yclaim = yarea;
		}
		else{
			if(yarea <= World.CLAIM[2])
				yclaim = World.CLAIM[2];
			if(yarea >= World.CLAIM[3])
				yclaim = World.CLAIM[3];
		}
		ret = Math.sqrt(Math.pow(xclaim-xarea, 2)+Math.pow(yclaim-yarea, 2));
		return ret;
	}

	/**
	 * Calculate the distance between an Area and the release zone
	 * 
	 * @param ar
	 * 			The area
	 * @return ret
	 */
	private double distanceToRelease(Area ar){
		double ret = Double.MAX_VALUE;
		double xrelease = 0;
		double yrelease = 0;
		double xarea = new Double(ar.getX());
		double yarea = new Double(ar.getY());
		if(xarea >= World.RELEASE[0] && xarea <= World.RELEASE[1]){
			xrelease = xarea;
		}
		else{
			if(xarea <= World.RELEASE[0])
				xrelease = World.RELEASE[0];
			if(xarea >= World.RELEASE[1])
				xrelease = World.RELEASE[1];
		}


		if(yarea >= World.RELEASE[2] && yarea <= World.RELEASE[3]){
			yrelease = yarea;
		}
		else{
			if(yarea <= World.RELEASE[2])
				yrelease = World.RELEASE[2];
			if(yarea >= World.RELEASE[3])
				yrelease = World.RELEASE[3];
		}
		ret = Math.sqrt(Math.pow(xrelease-xarea, 2)+Math.pow(yrelease-yarea, 2));
		return ret;
	}

	/**
	 * Add an Area to the critical queue depending of the state of the robot
	 * 
	 * @param ar
	 * 			The area to add
	 */
	private void addToQueue(Area ar){
		if(this.isCarrying){
			if(this.carryingQueue.size()==10){
				this.carryingQueue.poll();
			}
			this.carryingQueue.offer(ar);
		}
		else{
			if(this.claimQueue.size()==10){
				this.claimQueue.poll();
			}
			this.claimQueue.offer(ar);
		}
	}


}
