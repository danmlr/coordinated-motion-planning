
	/**
	 * Main program that takes as input a JSON storing the set of starting and target positions of robots
	 * and computes a solution to the coordinated motion problem, minimizing the 'total distance'
	 * 
	 * @author Luca Castelli Aleardi (Ecole Polytechnique, INF421, dec 2020)
	 */
public class OptimizeDistance {
	
	public static void main(String[] args) {
		System.out.println("Distance optimization (CG:SHOP 2021 contest)\n");
		if(args.length<1) {
			System.out.println("Error: one argument required: input file in JSON format");
			System.exit(0);
		}

		String inputFile=args[0]; // input file storing the input instance
		System.out.println("Input file: "+inputFile);
		if(inputFile.endsWith(".json")==false) {
			System.out.println("Error: wrong input format");
			System.out.println("Supported input format: JSON format");
			System.exit(0);
		}

		Instance input=IO.loadInputInstance(inputFile); // read the input file
		System.out.println(input);

		MotionAlgorithm algo=new MyBestAlgorithm(input); 
		algo.run(); // compute a solution for the input instance
		
		Solution solution=algo.getSolution();
		System.out.println(solution); // print the statistics
		IO.saveSolutionToJSON(solution, input.name+"_distance.json"); // export the solution in JSON format
	}

}
