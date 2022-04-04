/**
 * A class defining an instance of the problem containing: <br>
 * -) the starting positions of all robots <br>
 * -) the target positions of all robots <br>
 * -) the locations of obstacles (if any)
 * 
 * @author Luca Castelli Aleardi (INF421, Ecole Polytechnique, nov 2020)
 *
 */
public class Instance {
    /** coordinates of the bounding box containing the input robots and target positions */
    public int xmin=Integer.MAX_VALUE, xmax=Integer.MIN_VALUE, ymin=Integer.MAX_VALUE, ymax=Integer.MIN_VALUE;
    
    /** number of robots */
    int n;
    /** Name of the input instance */
    String name;
    
    /** array storing the input starting locations of robots */
    public Coordinates starts;
    /** array storing the input starting locations of targets */
    public Coordinates targets;
    /** array storing the input starting locations of obstacles: the set of obstacles could be empty */
    public Coordinates obstacles;
    
    public Instance(String name, Coordinates starts, Coordinates targets, Coordinates obstacles) {
    	this.name=name;
    	this.starts=starts;
    	this.targets=targets;
    	this.obstacles=obstacles;
    	n=starts.size();
    	
    	this.getBoundingBox();
    }
    
    /**
     * Compute and store the bounding box containing all robots/targets/obstacles. <br>
     * Results are stored in the 'xmin/xmax/ymin/ymax' fields of this class.
     */
    public void getBoundingBox() {
    	for(int i=0;i<n;i++) {
    		xmin=Math.min(xmin, starts.getX(i));
    		xmax=Math.max(xmax, starts.getX(i));
    		ymin=Math.min(ymin, starts.getY(i));
    		ymax=Math.max(ymax, starts.getY(i));
    		xmin=Math.min(xmin, targets.getX(i));
    		xmax=Math.max(xmax, targets.getX(i));
    		ymin=Math.min(ymin, targets.getY(i));
    		ymax=Math.max(ymax, targets.getY(i));
    	}
    	if(this.obstacles!=null) {
        	for(int i=0;i<this.obstacles.size();i++) {
        		xmin=Math.min(xmin, obstacles.getX(i));
        		xmax=Math.max(xmax, obstacles.getX(i));
        		ymin=Math.min(ymin, obstacles.getY(i));
        		ymax=Math.max(ymax, obstacles.getY(i));
        	}   		
    	}
    }
    
    public String toString() {
    	String result="Instance of a coordinate robot motion planning problem:\n";
    	result=result+"\tnumber of robots="+this.n+"\n";
    	if(obstacles!=null)
    		result=result+"\tnumber of obstacles="+this.obstacles.size()+"\n";
    	else
    		result=result+"\tnumber of obstacles=0\n";
    	result=result+"\tbounding box= ["+this.xmin+", "+this.xmax+"]x["+this.ymin+", "+this.ymax+"]\n";
    	
    	return result;
    }
        
}
