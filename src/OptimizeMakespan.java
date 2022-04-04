/**
	 * Main program that takes as input a JSON storing the set of starting and target positions of robots
	 * and computes a solution to the coordinated motion problem, minimizing the 'makespan'
	 * 
	 * @author Luca Castelli Aleardi (Ecole Polytechnique, INF421, dec 2020)
	 */

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;
import java.util.ArrayList;


public class OptimizeMakespan {
	
	public static Instance input; 
	
	static int pmin = 0; 
	static int pmax = 600;
	static int psteps = 10; 
	static int Niter = 20;
	static int Nsamples = 5;
	static int bestSolutionFound = 1000000;
	static boolean premierTour; 
	
	static int[] trial = new int[pmax+1];
	static Double[] lambda = new Double[pmax+1]; 
	static Integer[] theta = new Integer[pmax+1];
	static ArrayList<Integer>[] data = new ArrayList[pmax+1]; 
	
	
	public static double calcDebutLoi( double lambdaE, int thetaE, double succE) { 
		//Calcule la probabilité d'obtenir mieux que la meilleure solution étant donné les paramètres estimés
		int res = 0; 
		int f = 1; 
		for (int k =thetaE;k<=bestSolutionFound;k++) { 
			res += Math.exp(-lambdaE)*Math.pow(lambdaE, k-thetaE)/f;
			f = f * (k-thetaE+1); 
		}
		return res*succE ;
	}
	
	
	public static double expectedSuccess(int q) { 
		
		if (premierTour) { 
			return 1.0;
		}
		
		if (lambda[q]!=null) { 
			return calcDebutLoi( lambda[q],theta[q],data[q].size()/((double)trial[q]));			
		}
		
		int d = q ; 
		int g = q; 
		while(d<pmax+1 && lambda[d]==null  ) { 
			d = d+1 ;
		}
		while( g>pmin-1 &&lambda[g]==null ) { 
			g = g-1 ;
		}
		
		if (d==pmax+1) { 
			return expectedSuccess(g);
		}
		if (g==pmin-1) { 
			return expectedSuccess(d);
		}
		
		double lambdaE = lambda[g]+(lambda[d]-lambda[g])*(q-g)/((double)(d-g));
		int thetaE = (int)(theta[g]+(theta[d]-theta[g])*(q-g)/((double)(d-g)));
		
		double succg = data[g].size()/trial[g];
		double succd = data[d].size()/trial[d]; 
		
		double succE = succg+(succd-succg)*(q-g)/((double)(d-g));
		return calcDebutLoi(lambdaE,thetaE,succE);
		
		//Attention aux divisions entières !!! 
		
	}
	
	
	
	
	
	public static void main(String[] args) throws IOException {
		
		
		
		
		bestSolutionFound = 1000000;
		
		
		
		System.out.println("Makespan optimization (CG:SHOP 2021 contest)\n");
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

		input=IO.loadInputInstance(inputFile); // read the input file
		System.out.println(input);
		
		
		for (int i = 0; i < pmax+1; i++) { 
            data[i] = new ArrayList<Integer>(); 
        } 
		
		
		int dmaxRobotCible; 
		BoxEtRembobinage algo;
		
		
		
		////////////////////// Fin de l'initialisation 
		
		
		Random random = new Random(); 
		
		int nbSteps; 
		boolean calculReussi;
		
		int p = pmin + random.nextInt(pmax-pmin+1);
		premierTour =true; 
		
		while (true) { 
			
			for(int i =0; i<Nsamples; i++) { 
				//Exploration de p 
				for(int j=0;j<Niter;j++) {
						
						trial[p] = trial[p] + 1 ; 
						//EXecuter p
						algo=new BoxEtRembobinage(input); 
						algo.p = p;
						algo.run(); // compute a solution for the input instance
						
						try
						{
							// Ecriture log tests 
							FileWriter fw = new FileWriter(input.name+"_logHistory.csv",true);
							fw.write(p+";"+algo.solutionFound+";"+algo.getSolution().steps.size()+"\n");
							fw.close();
						}
						catch(IOException ioe)
						{
						 System.err.println(ioe.getMessage());
						}
						
						nbSteps = algo.getSolution().steps.size();
						calculReussi = algo.solutionFound;
						Solution solution=algo.getSolution();
						System.out.println(solution); // print the statistics
						System.out.println("p="+p);
						
						if (algo.solutionFound && solution.steps.size()<bestSolutionFound) { 
							IO.saveSolutionToJSON(solution, input.name+"_makespan.json"); // export the solution in JSON format
							bestSolutionFound= solution.steps.size();
							
							dmaxRobotCible = algo.dmaxRobotCible;
						
					//Fin execution 
						} 
						
						if (calculReussi) { 
							data[p].add(nbSteps);
						}
				}
				
				
				
				//Choix du mouvement à faire 
				int q = pmin + random.nextInt(pmax-pmin+1);
				double u = random.nextDouble();
				double pi = expectedSuccess(p);
				while (u*pi>expectedSuccess(q)) { 
					q = pmin + random.nextInt(pmax-pmin+1);
					u = random.nextDouble();		
				}
				p = q;
				
			}
			//Reevaluation des paramètres 
			premierTour=false; 
			for (int q=pmin;q<=pmax;q++) { 
				if (!data[q].isEmpty()) {
					double m = 0; 
					for (int k=0;k<data[q].size();k++) { 
						m += data[q].get(k);
					}
					//On estime la moyenne 
					m = m/data[q].size();
					double v =0; 
					
					for (int k=0;k<data[q].size();k++) { 
						int d = data[q].get(k);
						v += (d-m)*(d-m);
					}
					//On estime la variance 
					v = v /(data[q].size()-1);
					lambda[q]=v;
					theta[q]= (int)(m - v) ; 
					
				}
			}
			
			
			
			
			
			
		}
		
		
		
		
		
		
	}
}
	

	
