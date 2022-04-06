/**
 * An algorithm that computes a solution of the motion planning problem.
 * 
 * @author Gonzague de Carpentier (Ecole Polytechnique) 
 * @author Dan Meller (Ecole Polytechnique)
 */

public class BoxEtRembobinage extends MotionAlgorithm {
	
	/**
	 * CHAMPS ET INITIALISATION --------------------------------------------------------------------------------------------------------
	 */
	
	//Paramètres 
	public int p = 5;
	int tailleMax = 2; //taille maximum des box
	int limite1 = 300; 
	int limite2 = 300;
	boolean SAVE= false;
	
	/** An input instance of the motion planning problem **/
	public Instance input;
	
	/** The solution computed by the algorithm **/
	public Solution solution;
	
	/** Number of robots **/
	public int n;
	
	public boolean solutionFound = false;
	
	//distance maximale entre un robots et sa cible 
	public int dmaxRobotCible; 
	
	public BoxEtRembobinage(Instance input) {
		this.input=input;
		this.solution=new Solution(input.name); // create an empty solution (no steps at the beginning)
		n = input.n;
	}
	
	/**
	 * Return the current solution: it assumes that the solution has been computed
	 */
	public Solution getSolution() {
		return this.solution;
	}
	
	/**
	 * FONCTIONS PRINCIPALES-----------------------------------------------------------------------------------------------------------
	 */
	
	/**
	 * Compute a complete solution to the input problem: compute all steps, until all robots reach their target destinations
	 */
	public void run() {
		byte[] step;
		Coordinates targets;
		
	    //Dilatation des cibles 
		Instance input0 = new Instance(input.name, input.starts, input.targets, input.obstacles);
	    MoveTargets2 mt = new MoveTargets2(input0);
	    mt.p = p;
	    mt.tailleMax = tailleMax;
	    mt.run();
	    
	    targets = mt.targets;
	    
	    Instance input1 = new Instance(input.name, input.starts, targets, input.obstacles);
	    FirstAlgorithm algo1 = new FirstAlgorithm(input1);
	    algo1.limite = limite1;
	    algo1.run();
	    
	    dmaxRobotCible = algo1.dmaxRobotCible;
	    
	    Instance input2 = new Instance(input.name, input.targets, algo1.current, input.obstacles);
	    FirstAlgorithm algo2 = new FirstAlgorithm(input2);
	    algo2.limite = limite2;
	    algo2.run();
	    
	    // Block de presave pour debugger 
	    if (SAVE) { 
	    IO.saveSolutionToJSON(mt.solution, input.name + "_mt.json");
	    IO.saveSolutionToJSON(algo1.solution, input.name + "_algo1.json");
	    IO.saveSolutionToJSON(algo2.solution, input.name + "_algo2.json");
	    }
	    
	    /**
	    if (algo2.kebloPendant >= algo2.kebloMax && algo2.dmin <= 15) {
	    	Instance input3 = new Instance(input.name, algo2.current, algo1.current, input.obstacles);
	    	FirstAlgorithm3 algo3 = new FirstAlgorithm3(input3, algo2.robots);
		    algo3.run();
		    for (int i = algo3.solution.steps.size() - 1; i >=0 ; i--) {
		    	step = algo3.solution.steps.get(i);
		    	algo2.solution.addStep(step);
		    }
		    if (algo3.tousArrives)
		    	algo2.tousArrives = true;
	    }
	    
	    IO.saveSolutionToJSON(mt.solution, input.name + "_mt.json");
	    IO.saveSolutionToJSON(algo1.solution, input.name + "_algo1.json");
	    IO.saveSolutionToJSON(algo2.solution, input.name + "_algo2.json");
	    **/
	    
	    solution = algo1.solution;
	    
	    for (int i = algo2.solution.steps.size() - 1; i >=0 ; i--) {
	    	step = algo2.solution.steps.get(i);
	    	rotation180(step);
	    	solution.addStep(step);
	    }
		
	    if (algo2.tousArrives) {
	    	System.out.println("Solution computed");
	    	solutionFound = true;
	    } else {
	    	System.out.println("Failed to compute solution");
	    }
	}
	
	@Override
	public void computeOneStep() {
		// TODO Auto-generated method stub
		
	}
	
	/**
	 * FONCTIONS SECONDAIRES-----------------------------------------------------------------------------------------------------------
	 */
	
	
	public static byte rotationD(byte mov) { 
		switch(mov) { 
			case 0 :  
				return 0;
				
			case 1 : 
				return 3 ;
			case 2:
				return 4;
			case 3: 
				return 2;
			case 4:
				return 1;
			default : 
				return 0;
		}
	}
	
	public static byte rotation180(byte mov) { 
		return rotationD(rotationD(mov));
	}
	
	public static void rotation180(byte[] step) {
		for (int i = 0; i < step.length; i++) {
			step[i] = rotation180(step[i]);
		}
	}
	
	public static byte rotationG(byte mov) { 
		return rotationD(rotation180(mov));
	}
	
	public static int[] byteToMotion(byte mov) { 
		//byte est le type de stockage des mouvements dans la solution, motion[0] contient dx, motion[1] contient dy 
		int[] motion = new int[2];
		motion[0]=0;
		motion[1]=0;
		if (mov == 0) { 
			return motion;
		}
		int direction = 1 - (mov-1)/2;
		int sens = 2*(mov%2)-1;
		motion[direction]= sens;
		return motion; 
	}
	
	
	public static byte motionToByte(int[] motion) { 
		if (motion[1]!=0) {
			return (byte) (1 - (motion[1]-1)/2) ;
		}
		if (motion[0]!=0) {
			return (byte) (3 - (motion[0]-1)/2) ;
		}
		return (byte) 0;
	}
}
