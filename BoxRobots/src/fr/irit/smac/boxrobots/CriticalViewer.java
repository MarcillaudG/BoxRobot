package fr.irit.smac.boxrobots;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Stroke;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;

import fr.irit.smac.amak.Agent;
import fr.irit.smac.amak.Schedulable;
import fr.irit.smac.amak.Scheduler;
import fr.irit.smac.amak.Scheduling;
import fr.irit.smac.amak.ui.MainWindow;

public class CriticalViewer implements Schedulable {

	/**
	 * 
	 */
	private static final long serialVersionUID = 7895752986790657855L;

	/**
	 * Drawable canvas
	 */
	private JPanel canvas;

	/**
	 * Scheduler for the drawing frame
	 */
	private Scheduler scheduler;

	/**
	 * Unique index for giving unique id to each drawable ui
	 */
	private static int uniqueIndex;

	/**
	 * Unique id of the drawable ui
	 */
	private final int id = uniqueIndex++;
	
	private JFrame frame;

	protected Storehouse _storehouse;

	private ArrayList<Agent<?, World>> robots;

	/**
	 * Create and initialize the frame and the canvas
	 * 
	 * @param _scheduling
	 *            the scheduling mode
	 */
	public CriticalViewer() {
		onInitialConfiguration();
		canvas = new JPanel() {
			protected void paintComponent(java.awt.Graphics g) {

				Image buffer = createImage(800, 600);
				Graphics graphics = buffer.getGraphics();
				graphics.setColor(Color.BLACK);
				graphics.fillRect(0, 0, 800, 600);
				onDraw((Graphics2D) graphics);
				g.drawImage(buffer, 0, 0, null);
			}
		};
		canvas.addMouseListener(new MouseListener() {

			@Override
			public void mouseReleased(MouseEvent e) {
			}

			@Override
			public void mousePressed(MouseEvent e) {
			}

			@Override
			public void mouseExited(MouseEvent e) {
			}

			@Override
			public void mouseEntered(MouseEvent e) {

			}

			@Override
			public void mouseClicked(MouseEvent e) {
				onClick(e.getX(), e.getY());
			}
		});
		canvas.addMouseMotionListener(new MouseMotionListener() {

			@Override
			public void mouseMoved(MouseEvent e) {

			}

			@Override
			public void mouseDragged(MouseEvent e) {
				onMouseDragged(e.getX(), e.getY());
			}
		});

		canvas.setIgnoreRepaint(true);
		canvas.setPreferredSize(new Dimension(800, 650));
		this.frame = new JFrame("View of critical zone");
		this.frame.setPreferredSize(new Dimension(800, 650));
		this.frame.setMinimumSize(new Dimension(800, 650));
		this.frame.add(this.canvas);
		this.frame.setVisible(true);

		scheduler = new Scheduler(this);

	}

	/**
	 * This method is called at the very beginning of the DrawableUI creation.
	 * Any configuration should be made here.
	 */
	protected void onInitialConfiguration() {
	}

	@Override
	public final void cycle() {
		canvas.repaint();

	}

	/**
	 * This method is called when the canvas must be drawn again
	 * 
	 * @param graphics2D
	 *            Object used for drawing on the canvas
	 */
	protected  void onDraw(Graphics2D graphics2d){
		Stroke s = graphics2d.getStroke();
		if(_storehouse != null){
			graphics2d.setColor(new Color(165,165,165));
			graphics2d.fillRect(0, 0, 800, 600);
			for(int i = 0; i < World.HEIGHT; i++){
				for(int j = 0; j < World.WIDTH;j++){
					Area ar = this._storehouse.getEnvironment().getAreas()[i][j];
					int critic = ar.getCritic();
					if(critic > 40){
						critic = 40;
					}
					graphics2d.setColor(new Color(200,200-5*critic,200-5*critic));
					graphics2d.fillRect((int)RobotViewer.discreteToTopContinuous(ar.getX()) , (int)RobotViewer.discreteToTopContinuous(ar.getY()),
							RobotViewer.AREA_SIZE, RobotViewer.AREA_SIZE);
				}
			}
			for(int i = 0; i < World.HEIGHT; i++){
				for(int j = 0 ; j < World.WIDTH; j++){
					Area ar = _storehouse.getEnvironment().getAreas()[i][j];
					if(ar.getIsWall()){
						graphics2d.setColor(new Color(80,80,80));
						graphics2d.fillRect((int)RobotViewer.discreteToTopContinuous(ar.getX()) , (int)RobotViewer.discreteToTopContinuous(ar.getY()),
								RobotViewer.AREA_SIZE, RobotViewer.AREA_SIZE);
					}
					if(ar.getIsBox()){
						graphics2d.setColor(Color.BLUE);
						graphics2d.fillRect((int)RobotViewer.discreteToTopContinuous(ar.getX()) , (int)RobotViewer.discreteToTopContinuous(ar.getY()),
								RobotViewer.AREA_SIZE, RobotViewer.AREA_SIZE);
					}
				}
			}
			// Draw agents
			ArrayList<Agent<?, World>> agents = new ArrayList<>(_storehouse.getAgents());
			this.robots = agents;
			graphics2d.setColor(Color.RED);
			graphics2d.setStroke(s);
			

			//Draw the claim zone
			graphics2d.setColor(Color.BLUE);
			if(!(World.CLAIM[2] == 0)){
				graphics2d.drawLine(World.CLAIM[0]*RobotViewer.AREA_SIZE, (World.CLAIM[2]-1)*RobotViewer.AREA_SIZE,
						(World.CLAIM[1])*RobotViewer.AREA_SIZE, (World.CLAIM[2]-1)*RobotViewer.AREA_SIZE);
			}
			if(World.CLAIM[3] < 600){
				graphics2d.drawLine(World.CLAIM[0]*RobotViewer.AREA_SIZE, (World.CLAIM[3]+1)*RobotViewer.AREA_SIZE,
						(World.CLAIM[1])*RobotViewer.AREA_SIZE, (World.CLAIM[3]+1)*RobotViewer.AREA_SIZE);
			}
			if(!(World.CLAIM[0] == 0)){
				graphics2d.drawLine((World.CLAIM[0]-1)*RobotViewer.AREA_SIZE, (World.CLAIM[2]-1)*RobotViewer.AREA_SIZE,
						(World.CLAIM[0]-1)*RobotViewer.AREA_SIZE, (World.CLAIM[3])*RobotViewer.AREA_SIZE);
			}
			if(World.CLAIM[1] < 800){
				graphics2d.drawLine(World.CLAIM[1]*RobotViewer.AREA_SIZE, (World.CLAIM[2]-1)*RobotViewer.AREA_SIZE,
						World.CLAIM[1]*RobotViewer.AREA_SIZE, (World.CLAIM[3]+1)*RobotViewer.AREA_SIZE);
			}

			graphics2d.setColor(Color.ORANGE);
			//Draw the release zone
			if(!(World.RELEASE[2] == 0)){
				graphics2d.drawLine((World.RELEASE[0]-1)*RobotViewer.AREA_SIZE, (World.RELEASE[2]-1)*RobotViewer.AREA_SIZE,
						(World.RELEASE[1])*RobotViewer.AREA_SIZE, (World.RELEASE[2]-1)*RobotViewer.AREA_SIZE);
			}
			if(World.RELEASE[3] < 600){
				graphics2d.drawLine((World.RELEASE[0]-1)*RobotViewer.AREA_SIZE, (World.RELEASE[3]+1)*RobotViewer.AREA_SIZE,
						(World.RELEASE[1])*RobotViewer.AREA_SIZE, (World.RELEASE[3]+1)*RobotViewer.AREA_SIZE);
			}
			if(!(World.RELEASE[0] == 0)){
				graphics2d.drawLine((World.RELEASE[0]-1)*RobotViewer.AREA_SIZE, (World.RELEASE[2]-1)*RobotViewer.AREA_SIZE,
						(World.RELEASE[0]-1)*RobotViewer.AREA_SIZE, (World.RELEASE[3]+1)*RobotViewer.AREA_SIZE);
			}
			if(World.RELEASE[1] < 800){
				graphics2d.drawLine(World.RELEASE[1]*RobotViewer.AREA_SIZE, (World.RELEASE[2]-1)*RobotViewer.AREA_SIZE,
						World.RELEASE[1]*RobotViewer.AREA_SIZE, (World.RELEASE[3]+1)*RobotViewer.AREA_SIZE);
			}
		}
	}

	/**
	 * This method is called when the mouse is clicked on the canvas
	 * 
	 * @param x
	 *            X position of the mouse
	 * @param y
	 *            Y position of the mouse
	 */
	protected void onClick(int x, int y) {

	}

	/**
	 * This method is called when the mouse is dragged on the canvas
	 * 
	 * @param x
	 *            X position of the mouse
	 * @param y
	 *            Y position of the mouse
	 */
	protected void onMouseDragged(int x, int y) {

	}

	/**
	 * This method gives access to the scheduler of the DrawableUI
	 * 
	 * @return the scheduler
	 */
	public Scheduler getScheduler() {
		return scheduler;
	}

	/**
	 * Helper method to launch the scheduler
	 */
	public void start() {
		getScheduler().start();
	}
}