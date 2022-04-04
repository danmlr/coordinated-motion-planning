/**
 * This class encapsulates an array storing 2D integer coordinates (for representing robots, obstacles and target locations)
 * 
 * @author Luca Castelli Aleardi (Ecole Polytechnique, INF421, dec 2020)
 */
public class Coordinates {

    /** array storing 'n' 2D integer coordinates: <br>
     * e.g.  for the i-th object (e.g. robot or target)
     * pos[0][i] stores the 'x' coordinate, pos[1][i] contains its 'y' coordinate 
     * */
    private int[][] pos;
    
    /** size of the array: length of the rows */
    int n;
    
    /**
     * Initialize an array of size 'n'. All coordinates are set to (0, 0) by default.
     * 
     * @param n  number of objects in the array (e.g. robots or targets)
     */
    public Coordinates(int n) {
    	this.n=n;
    	this.pos=new int[2][n];
    }
    
    /**
     * Initialize an array, making a COPY of the input coordinates stored in the array 't'
     * 
     * @param t  input coordinates
     */
    public Coordinates(int[][] t) {
    	this.n=t[0].length;
    	this.pos=new int[2][n];
    	for(int i=0;i<n;i++) {
    		this.pos[0][i]=t[0][i];
    		this.pos[1][i]=t[1][i];
    	}
    }
    
    /**
     * Return the number of objects (e.g. robots, obstacles, or targets)
     * @return the size of the array
     */
    public int size() {
    	return this.n;
    }

    /**
     * Return the entire array storing the positions of all objects
     */
    public int[][] getPositions() {
    	return this.pos;
    }

    /**
     * Return the x-coordinate of an object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     * @return  the x coordinate
     */
    public int getX(int i) {
    	return this.pos[0][i];
    }

    /**
     * Return the y-coordinate of an object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     * @return  the y coordinate
     */
    public int getY(int i) {
    	return this.pos[1][i];
    }

    /**
     * Set the x-coordinate of the i-th object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     * @param x  value of the x coordinate
     */
    public void setX(int i, int x) {
    	this.pos[0][i]=x;
    }

    /**
     * Set the y-coordinate of the i-th object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     * @param y  value of the y coordinate
     */
    public void setY(int i, int y) {
    	this.pos[1][i]=y;
    }

    /**
     * Increase by 1 the x coordinate of an object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     */
    public void increaseX(int i) {
    	this.pos[0][i]++;
    }

    /**
     * Increase by 1 the y coordinate of an object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     */
    public void increaseY(int i) {
    	this.pos[1][i]++;
    }

    /**
     * Decrease by 1 the x coordinate of an object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     */
    public void decreaseX(int i) {
    	this.pos[0][i]--;
    }

    /**
     * Decrease by 1 the y coordinate of an object
     * 
     * @param i  index of the i-th robot, obstacle, or target
     */
    public void decreaseY(int i) {
    	this.pos[1][i]--;
    }

}
