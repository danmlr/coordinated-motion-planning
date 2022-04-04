/**
 * An abstract class defining a generic algorithm that computes a solution of the motion planning problem. <br>
 * 
 * @author Luca Castelli Aleardi (INF421, Ecole Polytechnique, dec 2020)
 *
 */
public abstract class MotionAlgorithm {
	
	/**
	 * Return the current solution: it assumes that the solution has been computed
	 */
	public abstract Solution getSolution();

	/**
	 * Compute a complete solution to the input problem: compute all steps, until all robots reach their target destinations
	 */
	public abstract void run();
	
	/**
	 * Add a new motion step to the current solution
	 */
	public abstract void computeOneStep();

	/**
	 * Perform one step of the parallel-motion for all robots: robots are move to new locations. <br>
	 * 
	 * Remark: there is no assumption on the motion step, so the new robot locations could be NOT VALID 
	 * (e.g. there could be conflicts between robots)
	 * 
	 * @param current  the current locations of all robots
	 * @param oneStepMove  the movements of all robots to be performed (each robot can move of one unity in one direction, or stay fixed)
	 * @return  a new array of size int[n][2] storing the new locations of all robots
	 */
	public static Coordinates moveRobotsOneStep(Coordinates current, byte[] oneStepMove) {
		if(current==null || oneStepMove==null)
			throw new Error("Error: input arguments not defined");
		
		int n=current.size();
		Coordinates newPos=new Coordinates(current.getPositions());
		
		for(int i=0;i<n;i++) {
			if(oneStepMove[i]==Solution.E) {
				newPos.increaseX(i); // increment 'x' by +1
			}
			else if(oneStepMove[i]==Solution.W) {
				newPos.decreaseX(i); // decrease 'x' by 1
			}
			else if(oneStepMove[i]==Solution.N) {
				newPos.increaseY(i); // increment 'y' by 1
			}
			else if(oneStepMove[i]==Solution.S) {
				newPos.decreaseY(i); // decrease 'y' by 1
			}
		}
		
		return newPos;		
	}

}
