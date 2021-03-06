package fr.irit.smac.boxrobots;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
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
import java.util.Hashtable;
import java.util.Random;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JToolBar;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

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
	
	private JCheckBox toward;
	
	private JCheckBox random;

	private Behavior currentBehavior;
	
	private JSlider slider;

	private ArrayList<Agent<?, World>> robots;

	/**
	 * The size of the areas
	 */
	public static final int AREA_SIZE = 10;


	protected Storehouse storehouse;

	private JCheckBox manageWall;

	private CriticalViewer criticalWindow;

	private CriticalViewer criticalClaimWindow;


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


		MainWindow.addMenuItem("manage box",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				boxing = !boxing;
				if(boxing)
					walling = false;
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
		

		MainWindow.addMenuItem("Random Maze",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				reset();
				storehouse.getEnvironment().randomMaze(2);
			}

		});
		

		MainWindow.addMenuItem("Random Path",  new ActionListener(){

			@Override
			public void actionPerformed(ActionEvent e) {
				reset();
				storehouse.getEnvironment().randomPath(2);
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
		

		this.toolbar = new JToolBar();
		this.toolbar.setLayout(new GridLayout(4, 2));

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


		this.coop = new JCheckBox("Mark change");
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

		this.toward = new JCheckBox("Carlo");
		this.toward.setSelected(true);
		this.toward.addItemListener(new ItemListener(){

			@Override
			public void itemStateChanged(ItemEvent arg0) {
				if(toward.isSelected()){
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.setBehavior(Behavior.CARLO);
						random.setSelected(false);
					}
				}
			}

		});
		this.toolbar.add(this.toward);

		this.random = new JCheckBox("random");
		this.random.addItemListener(new ItemListener(){

			@Override
			public void itemStateChanged(ItemEvent arg0) {
				if(random.isSelected()){
					for (Agent<?, World> agent : robots) {
						Robot robot = (Robot) agent;
						robot.setBehavior(Behavior.RANDOM);
						toward.setSelected(false);
					}
				}
			}

		});
		this.toolbar.add(this.random);
		

		this.manageWall = new JCheckBox("Wall");
		this.manageWall.addItemListener(new ItemListener(){

			@Override
			public void itemStateChanged(ItemEvent arg0) {
				if(manageWall.isSelected()){
					walling = true;
					boxing = false;
				}
				else{
					walling = false;
				}
			}

		});
		this.toolbar.add(this.manageWall);

		final Hashtable<Integer, JLabel> labelTable = new Hashtable<Integer, JLabel>();
		labelTable.put(new Integer(0), new JLabel("0"));
		labelTable.put(new Integer(20), new JLabel("20"));
		labelTable.put(new Integer(40), new JLabel("40"));
		labelTable.put(new Integer(60), new JLabel("60"));
		labelTable.put(new Integer(80), new JLabel("80"));
		labelTable.put(new Integer(100), new JLabel("100"));
		
		
		this.slider = new JSlider(0,100,20);
		this.slider.addChangeListener(new ChangeListener(){

			@Override
			public void stateChanged(ChangeEvent arg0) {
				for(Agent<?, World> agent: robots){
					((Robot)agent).setMemory(slider.getValue());
				}
			}
			
		});
		this.slider.setLabelTable(labelTable);
		this.slider.setPaintLabels(true);
		this.toolbar.add(this.slider);
		MainWindow.addToolbar(toolbar);
		
	}

	protected void reset() {
		for(Agent<?, World> agent: this.robots){
			this.storehouse._removeAgent(agent);
		}
		for(int i = 0; i < Storehouse.NB_ROBOT;i++){
			Robot rob = new Robot(storehouse,Behavior.CARLO);
			if(this.random.isSelected())
				rob.setBehavior(Behavior.RANDOM);
			if(this.coop.isSelected())
				rob.addModule(Module.COOP);
			if(this.memory.isSelected())
				rob.addModule(Module.MEMORY);
			if(this.direct.isSelected())
				rob.addModule(Module.DIRECT);
			this.storehouse._addAgent(rob);
		}
		while(storehouse.getCurrentNbBox() < Storehouse.NB_BOX)
			storehouse.releaseBox();
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
					graphics2d.setStroke(new BasicStroke(3));
					graphics2d.drawOval(((int) discreteToTopContinuous(robot.getX())), (int) discreteToTopContinuous(robot.getY()),
							AREA_SIZE, AREA_SIZE);
					graphics2d.setColor(Color.BLUE);
					graphics2d.fillRect((int)discreteToTopContinuous(robot.getX())+2 , (int)discreteToTopContinuous(robot.getY())+2,
							AREA_SIZE-3, AREA_SIZE-3);
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
		if(this.memory != null && this.memory.isSelected()){
			refreshMemory();
		}
	}

	private void refreshMemory() {
		if(this.criticalWindow == null){
			this.criticalWindow = new CriticalViewer(){

				protected void onInitialConfiguration() {
					this._storehouse = storehouse;
				}
			};
			this.criticalWindow.start();
		}
		else{
			this.criticalWindow.getScheduler().step();
		}
		

		if(this.criticalClaimWindow == null){
			this.criticalClaimWindow = new CriticalViewer(true){

				protected void onInitialConfiguration() {
					this._storehouse = storehouse;
				}
			};
			this.criticalClaimWindow.start();
		}
		else{
			this.criticalClaimWindow.getScheduler().step();
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
