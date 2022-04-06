/**
 * Ckeck the validity of a solution.
 * 
 * @author Luca Castelli Aleardi (Ecole Polytechnique, INF421, dec 2020)
 */

public class CheckSolution {
	
	public static void main(String[] args) {
		System.out.println("Checking solution (CG:SHOP 2021 contest)\n");
		if(args.length<2) {
			System.out.println("Error: two arguments required: input files in JSON format");
			System.exit(0);
		}

		String inputFile=args[0]; // input file storing the input instance
		String inputSolution=args[1]; // input file storing the input instance
		System.out.println("Input file: "+inputFile);
		System.out.println("Input solution: "+inputSolution);
		if(inputFile.endsWith(".json")==false || inputSolution.endsWith(".json")==false) {
			System.out.println("Error: wrong input format");
			System.out.println("Supported input format: JSON format");
			System.exit(0);
		}

		Instance input=IO.loadInputInstance(inputFile); // read the input file (problem instance)
		System.out.println(input);
		Solution solution=IO.loadSolution(inputSolution, input.n); // read the input solution
		System.out.println(solution);
		
		if(solution.isValid()) {
			System.out.println("The input solution is valid");
		}
		else {
			System.out.println("Error: the input solution is not a vaid solution for the input instance");
		}
	}

}
