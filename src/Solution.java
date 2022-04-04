import java.util.ArrayList;

/**
 * A class defining a solution to the coordinated (robot) motion planning problem. <br>
 * 
 * The parallel-motion steps of all robots are stored in an ArrayList<byte[]>, whose 'k-th' element is an array of size 'n'.<br>
 * 
 * @author Luca Castelli Aleardi (INF421, Ecole Polytechnique, nov 2020)
 *
 */

public class Solution {
	/** Possible robot movements into Western, Eastern, Northern or Southern direction. (FIXED means that the robot is not moving) */
	public final static byte FIXED=0, N=1, S=2, E=3, W=4;

	/** Name of the input instance */
	public String name;
	
	/** 
	 * A dynamic array storing all motion parallel steps: 'k-th' element is an array of size 'n' representing
	 * the displacement of all robots at time 'k'. <br>
	 * Remark: if the 'i-th' is not moving at time 'k' then the corresponding entry in the array is FIXED (equal to 0)
	 **/
	public ArrayList<byte[]> steps;
	
	/**
	 * Initialize the motion of robots (no steps at the beginning)
	 */
	public Solution(String name) {
		this.name=name;
		this.steps=new ArrayList<byte[]>(); // empty list of steps at the beginning
	}
	
	/**
	 * Add a new step to the current solution
	 */
	public void addStep(byte[] mov) {
		if(mov!=null) {
			this.steps.add(mov);
		}
	}
	
	public byte[] popStep() {
		byte[] lastMove = null;
		if (steps.size() > 0) {
			lastMove = steps.get(steps.size()-1);
			steps.remove(steps.size()-1);
		}
		return lastMove;
	}

	/**
	 * The makespan is just the number of parallel steps (e.g. the time until all robots have reached their destinations).
	 * 
	 * @return the makespan
	 */
	public int makespan() {
		return this.steps.size();
	}
	
	/**
	 * Compute and return the total distance traveled by all robots (until they have all reached their destinations).
	 * @return the total traveled distance of all robots (in all steps)
	 */
	public int getTotalDistance() {
		int result=0;
		for(byte[] moves: this.steps) {
			for(int i=0;i<moves.length;i++)
				if(moves[i]!=Solution.FIXED)
					result++;
		}
		return result;
	}
	
	/**
	 * Check whether the solution describe a valid trajectory for all robots and at any step.
	 * 
	 * @return TRUE is all motion parallel steps are valid (according to the rules of the problem)
	 */
	public boolean isValid() {
		throw new Error("TO BE COMPLETED");
	}
	
	public String toString() {
		String result="Solution to the input instance: "+this.name+"\n";
		result=result+"\tnumber of steps (makespan): "+this.makespan()+"\n";
		result=result+"\ttotal distance (total number of robot moves): "+this.getTotalDistance();
		return result;
	}
	
}
