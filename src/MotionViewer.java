import java.util.Collection;
import java.util.LinkedList;

import processing.core.PApplet;

/**
 * Main program that takes as input a JSON storing the set of starting and target positions of robots
 * 
 * @author Luca Castelli Aleardi (Ecole Polytechnique, INF421, nov 2020)
 */


public class MotionViewer extends PApplet {
	private static final int ROBOTS=0, TARGETS=1;

	// parameters for the rendering
	private int backgroundColor=255;

	// parameters of the 2d frame/canvas	
	public static int sizeX; // horizontal size of the canvas
	public static int sizeY; // vertical size of the canvas (pixels)
	public int sizeT; // size of label fonts
	private int cellSize; // size of the cell in the grid (expressed in pixels)
	private int shiftX, shiftY; // (pixels) coordinates of the bottom left vertex of the bounding box
	int[] currentPosition; // mouse position on the screen (pixels)
	private int drawObject=ROBOTS; // choose the object to draw: robots or targets

	/** Input instance */
	public static Instance input;	
	/** A solution to the input problem */
	public static Solution solution;	
	/** current time step 't' of the motion. The starting position correspond to the value 't=0' */
	public int timeStep;
	/** current robot locations, corresponding to all steps of the solution <br>
	 *  Remark: <br>
	 *  robotLocations[0] store the starting locations of all robots <br>
	 *  robotLocations[1] store the locations of all robots at time step 't=1' (e.g. after the first parallel move) <br>
	 *  robotLocations[i] store the locations of all robots at time step 't=i' (e.g. after 'i' parallel moves)
	 * */
	public Coordinates[] robotLocations;
	
	public int specialRobot=347;
	public int specialRobot2=66;
	
	public static void main(String[] args) {
		System.out.println("Tools for the \"CG:SHOP 2021 contest\"");
		if(args.length<1) {
			System.out.println("Error: one argument required: input file in JSON format");
			System.exit(0);
		}

		String inputFile, inputSolution;
		inputFile=args[0]; // input file storing the input instance
		System.out.println("Input file: "+inputFile);
		
		if(args.length==1)
			inputSolution=null; // no input solution
		else {
			inputSolution=args[1];
			System.out.println("Input solution: "+inputSolution);
		}
		
		if(inputFile.endsWith(".json")==false) {
			System.out.println("Error: wrong input format");
			System.out.println("Supported input format: JSON format");
			System.exit(0);
		}

		Instance input=IO.loadInputInstance(inputFile); // read the input file (problem instance)
		System.out.println(input);
		
		Solution solution;
		if(inputSolution!=null) {
			solution=IO.loadSolution(inputSolution, input.n); // read the input solution (if any)
			System.out.println(solution);
		}
		else { // input solution not defined
			solution=new Solution(input.name); // create an empty solution (without any step)
			System.out.println("Input solution: not defined");
		}

		// Initialize the Processing viewer
		MotionViewer.sizeX=800;
		MotionViewer.sizeY=600;
		MotionViewer.input=input;
		MotionViewer.solution=solution;
		PApplet.main(new String[] { "MotionViewer" }); // launch the Processing viewer
	}

	public void settings(){
		this.size(sizeX,sizeY); // set the size of the Java Processing frame

		int max=Math.max(this.input.xmax, this.input.ymax)+8;
		this.cellSize=(sizeX)/max; // adjust the size of cells
		this.cellSize=Math.min(this.cellSize, ((sizeY-100))/max);
		this.shiftX=(sizeX-(this.cellSize*(max-1)))/2;
		this.shiftY=((sizeY-100)-(this.cellSize*(this.input.ymax-1)))/2;
	}

	public void setup(){
		System.out.println("Running motion viewer");
		System.out.println("\t input problem: "+this.input.n+" robots");
		if(this.solution!=null) {
			System.out.println("\t input solution: "+this.solution.steps.size()+" time steps");
		}
		
		this.sizeT=this.cellSize-6;
		this.textSize(Math.max(sizeT, 1));
		
		this.timeStep=0; // at the beginning the time step is t=0 by default (robots are located at their initial positions)
		
		if(this.solution==null || this.solution.steps.size()==0) { // no solution defined (only starting and target positions are displayed)
			this.robotLocations=new Coordinates[1];
			this.robotLocations[0]=new Coordinates(this.input.starts.getPositions()); // robots are located at initial positions
		}
		else {
			int nSteps=this.solution.steps.size();
			this.robotLocations=new Coordinates[nSteps+1];
			this.robotLocations[0]=new Coordinates(this.input.starts.getPositions()); // initialize robot locations
			
			for(int i=0; i<nSteps;i++) { // given the solution, set the robot locations at each time step
				this.robotLocations[i+1]=MotionAlgorithm.moveRobotsOneStep(this.robotLocations[i], this.solution.steps.get(i));
			}
		}
	}

	/**
	 * Deal with keyboard events
	 */
	public void keyPressed(){
		switch(key) {
		case('d'): this.drawObject=(this.drawObject+1)%2; break;
		case('+'): this.cellSize+=1; break;
		case('-'): this.cellSize-=1; break;
		case('n'): this.getNextRobotLocation(); break;
		case('p'): this.getPreviousRobotLocation(); break;
		}
	}

	/** Move all robots to the next location, according to the current solution */
	public void getNextRobotLocation() {
		if(this.solution.steps.size()==0) {
			System.out.println("No solution defined (empty solution)");
		}
		if(this.timeStep>=this.solution.steps.size()) // no more time steps to show
			return;
		
		this.timeStep++;
		System.out.println("Moving robots: step "+(this.timeStep));
	}

	/** Move all robots to the previous location, according to the current solution */
	public void getPreviousRobotLocation() {
		if(this.solution.steps.size()==0) {
			System.out.println("No solution defined (empty solution)");
		}
		if(this.timeStep<=0) // no more time steps to show
			return;
		
		this.timeStep--;
		System.out.println("Moving robots: step "+(this.timeStep));
	}

	/**
	 * Main function for drawing all geometric objects (points, edges, ...)
	 */
	public void draw(){
		this.background(this.backgroundColor); // set the color of background (clean the background)

		if(this.drawObject==ROBOTS)
			this.drawRobots(this.robotLocations[this.timeStep]);
		else if(this.drawObject==TARGETS)
			this.drawTargets(this.input.targets);
		
		if(this.input.obstacles!=null)
			this.drawObstacles(this.input.obstacles);

		this.drawGrid(this.input.xmax, this.input.ymax);
		this.drawOptions();
	}

	/**
	 * Show options on the screen
	 */
	public void drawOptions() {
		String label="press '-' or '+' for zooming\n"; // text to show
		label=label+"press 'd' to switch between robots and targets\n";
		label=label+"press 'n' for the next time step\n";
		label=label+"press 'p' for the previous time step\n";
		label=label+"use 'right mouse button' to drag the layout (press right button, move the mouse and release)";

		int posX=0;
		int posY=0;
		int textHeight=100;
		this.textSize(12);

		//this.stroke(edgeColor, edgeOpacity);
		this.fill(150);
		this.rect((float)posX, (float)posY, this.width, textHeight); // fill a gray rectangle
		this.fill(0);
		this.text(label, (float)posX+2, (float)posY+10); // draw the text
	}

	public void mousePressed() {
		  this.currentPosition=new int[] {mouseX, mouseY};
	}

	public void mouseReleased() {
		  if(mouseButton==RIGHT) { // translate the window
			  int deltaX=(mouseX-currentPosition[0])/2;
			  int deltaY=(currentPosition[1]-mouseY)/2;
			  this.shiftX=this.shiftX+deltaX;
			  this.shiftY=this.shiftY+deltaY;

			  int[] currentPosition=new int[] {mouseX, mouseY};
		  }
	}

	/**
	 * Draw all robots
	 */
	public void drawRobots(Coordinates pos) {
		this.sizeT=this.cellSize-6;
		this.textSize(Math.max(this.sizeT, 1));
		
		for(int i=0;i<pos.size();i++) {
			if (i == specialRobot || i == specialRobot2) {
				this.drawTarget(pos.getX(i), pos.getY(i), i);
			} else {
				this.drawRobot(pos.getX(i), pos.getY(i), i);
			}
		}
		if(this.cellSize<10) // cells are too small: labels are NOT displayed
			return;
		
		this.fill(0);
		for(int i=0;i<pos.size();i++)
			this.drawLabel(""+i, pos.getX(i), pos.getY(i));
	}

	/**
	 * Draw all targets
	 */
	public void drawTargets(Coordinates pos) {
		this.sizeT=this.cellSize-6;
		this.textSize(Math.max(this.sizeT, 1));

		for(int i=0;i<pos.size();i++){
			if (i == specialRobot || i == specialRobot2) {
				this.drawRobot(pos.getX(i), pos.getY(i), i);
			} else {
				this.drawTarget(pos.getX(i), pos.getY(i), i);
			}
		}
		
		if(this.cellSize<10) // cells are too small: labels are NOT displayed
			return;
		
		this.fill(0);
		for(int i=0;i<pos.size();i++)
			this.drawLabel(""+i, pos.getX(i), pos.getY(i));
	}

	/**
	 * Draw all obstacles
	 */
	public void drawObstacles(Coordinates pos) {
		for(int i=0;i<pos.size();i++) {
			this.drawCell(pos.getX(i), pos.getY(i), 0, 0, 0); // obstacles are drawn as black cells
		}
	}

	/**
	 * Draw an integer label in a square
	 */
	public void drawLabel(String label, int x, int y) {
		int delta=1;
		if(label.length()>2)
			delta=0;

		int[] pos=this.getPixel(x-1, y-1);
		this.text(label, pos[0]+delta, pos[1]-delta); // draw the text
	}

	/**
	 * Draw a colored cell
	 */
	public void drawCell(int x, int y, int r, int g, int b) {
		this.fill(r, g, b); // robot color
		this.noStroke();

		int[] pos=this.getPixel(x-1, y);
		this.rect(pos[0], pos[1], this.cellSize, this.cellSize);
	}

	/**
	 * Draw a robot at a given location (x, y)
	 */
	public void drawRobot(int x, int y, int color) {
		int val=120;
		int c=(255-val)/(this.input.n);
		
		this.drawCell(x, y, val, val, 20+val+color*c); // robot color
	}

	/**
	 * Draw a target cell
	 */
	public void drawTarget(int x, int y, int color) {
		int c=(255-50)/(this.input.n);

		this.drawCell(x, y, 255, 0, 0); // robot color
	}

	/**
	 * Draw the entire grid
	 */
	public void drawGrid(int xmax, int ymax) {
		this.stroke(150, 255); // grid color
		this.strokeWeight(1);

		int min=-shiftX/this.cellSize;
		int max=Math.max(this.width/this.cellSize, Math.abs(shiftX))+1;
		for(int i=min;i<=max;i++) { // draw vertical lines
			int[] pixel=this.getPixel(i, 0);
			this.line(pixel[0], 0, pixel[0], this.height);
		}
		min=-shiftY/this.cellSize;
		max=Math.max(this.height/this.cellSize, Math.abs(shiftY))+1;
		for(int j=min;j<=max;j++) { // draw horizontal lines
			int[] pixel=this.getPixel(0, j);
			this.line(0, pixel[1], this.width, pixel[1]);
		}

		this.strokeWeight(2);
		this.stroke(0, 255); // border color
		// draw the bounding box containing all starting and target locations
		int[] pixel00=this.getPixel(-1, -1);
		int[] pixel10=this.getPixel(xmax, -1);
		int[] pixel11=this.getPixel(xmax, ymax);
		int[] pixel01=this.getPixel(-1, ymax);
		this.line(pixel00[0], pixel00[1], pixel10[0], pixel10[1]);
		this.line(pixel01[0], pixel01[1], pixel11[0], pixel11[1]);
		this.line(pixel00[0], pixel00[1], pixel01[0], pixel01[1]);
		this.line(pixel10[0], pixel10[1], pixel11[0], pixel11[1]);

		this.strokeWeight(1);
	}

	/**
	 * Given a point (x, y) on a regular integer grid whose cells have a given 'cellSize', returns the corresponding pixel on the screen
	 */
	public int[] getPixel(int x, int y) {
		int i=this.shiftX+(x*this.cellSize);
		int j=this.height-(this.shiftY+(y*this.cellSize));
		
		return new int[] {i, j};
	}
}
