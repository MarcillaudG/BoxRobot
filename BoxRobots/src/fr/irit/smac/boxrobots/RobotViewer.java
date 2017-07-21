package fr.irit.smac.boxrobots;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Polygon;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JCheckBox;
import javax.swing.JPanel;
import javax.swing.JToolBar;

import fr.irit.smac.amak.Agent;
import fr.irit.smac.amak.Scheduling;
import fr.irit.smac.amak.ui.DrawableUI;
import fr.irit.smac.amak.ui.MainWindow;
import fr.irit.smac.boxrobots.Robot.Behavior;
import fr.irit.smac.boxrobots.Robot.Module;


public class RobotViewer extends DrawableUI{

	private boolean walling = false;

	private boolean boxing = false;

	private RobotViewer instance = this;

	private Area previousArea;

	private JToolBar toolbar;

	private JCheckBox direct;

	private JCheckBox memory;

	private JCheckBox coop;

	private Behavior currentBehavior;

	private ArrayList<Agent<?, World>> robots;

	/**
	 * The size of the areas
	 */
	public static final int AREA_SIZE = 10;


	protected Storehouse storehouse;

	public RobotViewer() {
		super(Scheduling.UI);
		this.robots = new ArrayList<Agent<?, World>>();
		this.currentBehavior = Behavior.TOWARD;

		MainWindow.addMenuItem("Reset", new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent e) {
				reset();
			}

		});


		MainWindow.addMenuItem("Manage Wall", new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent e) {
				walling = !walling;
				boxing = false;
			}

		});
		MainWindow.addMenuItem("Manage Box", new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				walling = false;
				boxing = !boxing;
			}

		});

		MainWindow.addMenuItem("Toward",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				storehouse.setBehavior(Behavior.TOWARD);
				currentBehavior = Behavior.TOWARD;
			}

		});

		MainWindow.addMenuItem("Random",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				storehouse.setBehavior(Behavior.RANDOM);
				currentBehavior = Behavior.RANDOM;
			}

		});


		MainWindow.addMenuItem("Add Robot",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				Robot robot = new Robot(storehouse,currentBehavior);
				if(direct.isSelected())
					robot.addModule(Module.DIRECT);
				if(memory.isSelected())
					robot.addModule(Module.MEMORY);
				if(coop.isSelected())
					robot.addModule(Module.COOP);
			}

		});

		MainWindow.addMenuItem("Add Robot x 10",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				for(int i = 0; i < 10; i++){
					Robot robot = new Robot(storehouse,currentBehavior);
					if(direct.isSelected())
						robot.addModule(Module.DIRECT);
					if(memory.isSelected())
						robot.addModule(Module.MEMORY);
					if(coop.isSelected())
						robot.addModule(Module.COOP);
				}
			}

		});

		MainWindow.addMenuItem("Remove Robot",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				if(robots.size()>0)
					storehouse._removeAgent(robots.get(robots.size()-1));
			}

		});

		MainWindow.addMenuItem("Remove Robot x 10",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				if(robots.size()>10){
					int step = 0;
					for(int i = robots.size()-1; step < 10 ; i--){
						storehouse._removeAgent(robots.get(i));
						step++;
					}
				}
			}

		});
		this.getCanvas().addMouseListener(new MouseAdapter(){

			@Override
			public void mouseReleased(MouseEvent e){
				if(e.getX()>=0 && e.getX() <= 800 && e.getY()>=0 && e.getY() <= 600){
					if(walling){
						Area ar = storehouse.getEnvironment().getAreaByPosition(e.getX()/AREA_SIZE, e.getY()/AREA_SIZE);
						ar.setIsWall(!ar.getIsWall());
					}
					if(boxing){
						Area ar = storehouse.getEnvironment().getAreaByPosition(e.getX()/AREA_SIZE, e.getY()/AREA_SIZE);
						ar.setBox(!ar.getIsBox());
					}
				}
			}
		});

		this.getCanvas().addMouseMotionListener(new MouseMotionAdapter(){
			@Override
			public void mouseDragged(MouseEvent e){
				if(e.getX()>=0 && e.getX() < 800 && e.getY()>=0 && e.getY() < 600){
					if(walling){
						Area ar = storehouse.getEnvironment().getAreaByPosition(e.getX()/AREA_SIZE, e.getY()/AREA_SIZE);
						if(!ar.equals(previousArea)){
							ar.setIsWall(!ar.getIsWall());
							previousArea = ar;
						}
					}
					if(boxing){
						Area ar = storehouse.getEnvironment().getAreaByPosition(e.getX()/AREA_SIZE, e.getY()/AREA_SIZE);
						if(!ar.equals(previousArea)){
							ar.setBox(!ar.getIsBox());
							previousArea = ar;
						}
					}
				}
			}
		});

		//TODO
		this.toolbar = new JToolBar();

		this.direct = new JCheckBox("Keep Direction");
		this.direct.addItemListener(new ItemListener(){

			@Override
			public void itemStateChanged(ItemEvent arg0) {
				if(direct.isSelected()){
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.addModule(Module.DIRECT);
					}
				}
				else{
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.removeModule(Module.DIRECT);
					}
				}

			}

		});
		this.toolbar.add(this.direct);


		this.memory = new JCheckBox("Memory");
		this.memory.addItemListener(new ItemListener(){

			@Override
			public void itemStateChanged(ItemEvent arg0) {
				if(memory.isSelected()){
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.addModule(Module.MEMORY);
					}
				}
				else{
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.removeModule(Module.MEMORY);
					}
				}

			}

		});
		this.toolbar.add(this.memory);


		this.coop = new JCheckBox("Coop");
		this.coop.addItemListener(new ItemListener(){

			@Override
			public void itemStateChanged(ItemEvent arg0) {
				if(coop.isSelected()){
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.addModule(Module.COOP);
					}
				}
				else{
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.removeModule(Module.COOP);
					}
				}

			}

		});
		this.toolbar.add(this.coop);
		MainWindow.addToolbar(toolbar);
	}

	protected void reset() {
		for(Agent<?, World> agent: this.robots){
			this.storehouse._removeAgent(agent);
		}
		for(int i = 0; i < Storehouse.NB_ROBOT;i++){
			this.storehouse._addAgent(new Robot(storehouse,Behavior.TOWARD));
		}
	}

	@Override
	protected void onDraw(Graphics2D graphics2d) {
		Stroke s = graphics2d.getStroke();
		if(storehouse != null){
			graphics2d.setColor(new Color(165,165,165));
			graphics2d.fillRect(0, 0, 800, 600);
			for(int i = 0; i < World.HEIGHT; i++){
				for(int j = 0 ; j < World.WIDTH; j++){
					Area ar = storehouse.getEnvironment().getAreas()[i][j];
					if(ar.getIsWall()){
						graphics2d.setColor(new Color(80,80,80));
						graphics2d.fillRect((int)discreteToTopContinuous(ar.getX()) , (int)discreteToTopContinuous(ar.getY()),
								AREA_SIZE, AREA_SIZE);
					}
					if(ar.getIsBox()){
						graphics2d.setColor(Color.BLUE);
						graphics2d.fillRect((int)discreteToTopContinuous(ar.getX()) , (int)discreteToTopContinuous(ar.getY()),
								AREA_SIZE, AREA_SIZE);
					}
				}
			}
			// Draw agents
			ArrayList<Agent<?, World>> agents = new ArrayList<>(storehouse.getAgents());
			this.robots = agents;
			graphics2d.setColor(Color.RED);
			for (Agent<?, World> agent : agents) {
				Robot robot = (Robot) agent;
				if(robot.isCarrying()){
					graphics2d.setStroke(s);
					graphics2d.setColor(Color.ORANGE);
					graphics2d.fillOval(((int) discreteToTopContinuous(robot.getX())), (int) discreteToTopContinuous(robot.getY()),
							AREA_SIZE, AREA_SIZE);
				}
				else{
					graphics2d.setStroke(new BasicStroke(3));
					graphics2d.setColor(Color.GREEN);
					graphics2d.drawOval((int) discreteToTopContinuous(robot.getX()), (int) discreteToTopContinuous(robot.getY()),
							AREA_SIZE, AREA_SIZE);
				}
			}
			graphics2d.setStroke(s);
			/*for(Boxs box : storehouse.getBoxs()){
				graphics2d.fillRect((int)discreteToTopContinuous(box.getDx()) , (int)discreteToTopContinuous(box.getDy()),
						AREA_SIZE, AREA_SIZE);
				storehouse.getEnvironment().getAreaByPosition(box.getDx(), box.getDy()).setBox(true);
			}*/

			//Draw the claim zone
			graphics2d.setColor(Color.BLUE);
			if(!(World.CLAIM[2] == 0)){
				graphics2d.drawLine(World.CLAIM[0]*AREA_SIZE, (World.CLAIM[2]-1)*AREA_SIZE, (World.CLAIM[1])*AREA_SIZE, (World.CLAIM[2]-1)*AREA_SIZE);
			}
			if(World.CLAIM[3] < 600){
				graphics2d.drawLine(World.CLAIM[0]*AREA_SIZE, (World.CLAIM[3]+1)*AREA_SIZE, (World.CLAIM[1])*AREA_SIZE, (World.CLAIM[3]+1)*AREA_SIZE);
			}
			if(!(World.CLAIM[0] == 0)){
				graphics2d.drawLine((World.CLAIM[0]-1)*AREA_SIZE, (World.CLAIM[2]-1)*AREA_SIZE, (World.CLAIM[0]-1)*AREA_SIZE, (World.CLAIM[3])*AREA_SIZE);
			}
			if(World.CLAIM[1] < 800){
				graphics2d.drawLine(World.CLAIM[1]*AREA_SIZE, (World.CLAIM[2]-1)*AREA_SIZE, World.CLAIM[1]*AREA_SIZE, (World.CLAIM[3]+1)*AREA_SIZE);
			}

			graphics2d.setColor(Color.ORANGE);
			//Draw the release zone
			if(!(World.RELEASE[2] == 0)){
				graphics2d.drawLine((World.RELEASE[0]-1)*AREA_SIZE, (World.RELEASE[2]-1)*AREA_SIZE, (World.RELEASE[1])*AREA_SIZE, (World.RELEASE[2]-1)*AREA_SIZE);
			}
			if(World.RELEASE[3] < 600){
				graphics2d.drawLine((World.RELEASE[0]-1)*AREA_SIZE, (World.RELEASE[3]+1)*AREA_SIZE, (World.RELEASE[1])*AREA_SIZE, (World.RELEASE[3]+1)*AREA_SIZE);
			}
			if(!(World.RELEASE[0] == 0)){
				graphics2d.drawLine((World.RELEASE[0]-1)*AREA_SIZE, (World.RELEASE[2]-1)*AREA_SIZE, (World.RELEASE[0]-1)*AREA_SIZE, (World.RELEASE[3]+1)*AREA_SIZE);
			}
			if(World.RELEASE[1] < 800){
				graphics2d.drawLine(World.RELEASE[1]*AREA_SIZE, (World.RELEASE[2]-1)*AREA_SIZE, World.RELEASE[1]*AREA_SIZE, (World.RELEASE[3]+1)*AREA_SIZE);
			}

		}
	}

	/**
	 * Helper function aiming at converting a discrete value to a screen value
	 * 
	 * @param dx
	 * @return
	 */
	public static double discreteToTopContinuous(int dx) {
		return (dx) * AREA_SIZE;
	}

}
