import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import processing.data.JSONArray;
import processing.data.JSONObject;

/**
 * This class provides methods for dealing with input/output for JSON files
 * 
 * @author Luca Castelli Aleardi (INF421, Ecole Polytechnique, nov 2020)
 */
public class IO {
	
	/**
	 * Load an instance of the 'coordinated motion planning problem' from an input JSON file
	 * 
	 * @param filename  name of the input file
	 */
	public static Instance loadInputInstance(String filename){
		System.out.print("Reading JSON input file: "+filename+"...");
		JSONObject json;
		
		json = loadFile(filename);
		System.out.println("ok");
		
		String name = json.getString("name");
		JSONArray starts = json.getJSONArray("starts");
		JSONArray targets = json.getJSONArray("targets");
		JSONArray obstacles = json.getJSONArray("obstacles");
		if(targets.size()!=starts.size())
			throw new Error("Error: wrong number of start and target positions");
		
		int nRobots=starts.size();
		int[][] startLocations=new int[2][nRobots];
		int[][] targetLocations=new int[2][nRobots];
		int[][] obstacleLocations=new int[2][obstacles.size()];
		
		System.out.print("Reading start and target positions...");
		int[] loc;
		for(int i=0;i<nRobots;i++) {
			JSONArray position=starts.getJSONArray(i);
			loc=position.getIntArray();
			startLocations[0][i]=loc[0];
			startLocations[1][i]=loc[1];
			//System.out.println("robot"+i+" ["+loc[0]+", "+loc[1]+"]");
		}
		for(int i=0;i<nRobots;i++) {
			JSONArray position=targets.getJSONArray(i);
			loc=position.getIntArray();
			targetLocations[0][i]=loc[0];
			targetLocations[1][i]=loc[1];
		}
		for(int i=0;i<obstacles.size();i++) {
			JSONArray position=obstacles.getJSONArray(i);
			loc=position.getIntArray();
			obstacleLocations[0][i]=loc[0];
			obstacleLocations[1][i]=loc[1];
		}
		System.out.println("done");
		
		System.out.println("\tname: "+name);
		System.out.println("\tnumber of robots: "+nRobots);
		System.out.println("Input instance loaded from file\n------------------");
		
		return new Instance(name, new Coordinates(startLocations), new Coordinates(targetLocations), new Coordinates(obstacleLocations));
	}
	
	/**
	 * Load a solution of the 'coordinated motion planning problem' from a JSON file
	 * 
	 * @param filename  name of the input file
	 * 
	 */
	public static Solution loadSolution(String filename, int n){
		System.out.print("Reading solution from JSON file: "+filename+"...");
		JSONObject json;
		
		json = loadFile(filename);
		System.out.println("ok");
		
		String instance = json.getString("instance");
		JSONArray steps = json.getJSONArray("steps");
		int nSteps=steps.size();
		
		Solution solution=new Solution(instance);
		
		System.out.print("Reading moves...");
		for(int k=0;k<nSteps;k++) {
			JSONObject o=steps.getJSONObject(k);
			
			byte[] moves=new byte[n];
			for(int i=0;i<n;i++) {
				String m;
				if(o.get(""+i)!=null) {
					m=""+o.get(""+i);
					if(m.equals("W")) {
						moves[i]=4;
					}
					else if(m.equals("E")) {
						moves[i]=3;
					}
					else if(m.equals("S")) {
						moves[i]=2;
					}
					else if(m.equals("N")) {
						moves[i]=1;
					}
					//System.out.print(" robot "+i+": "+o.get(""+i));
				}
			}
			solution.addStep(moves);
		}
		System.out.println("done");
		
		System.out.println("\tinstance: "+instance);
		System.out.println("\tnumber of steps: "+nSteps);
		System.out.println("Input solution loaded from file\n------------------");
		
		return solution;
	}

	/**
	 * Load a JSON object from input file
	 * 
	 * @param filename name of the input file
	 */
	private static JSONObject loadFile(String filename) {
		JSONObject outgoing=null;
		BufferedReader reader = null;
		FileReader fr = null;

		try {
			fr = new FileReader(filename);
			reader = new BufferedReader(fr);
			outgoing = new JSONObject(reader);
		} catch (IOException e) {
			System.err.format("IOException: %s%n", e);
		} finally {
			try {
				if (reader != null)
					reader.close();

				if (fr != null)
					fr.close();
			} catch (IOException ex) {
				System.err.format("IOException: %s%n", ex);
			}
		}
		
		return outgoing;
	}

    /**
     * Output a solution of the motion planning problem to a JSON file
     * 
     */		   
    public static void saveSolutionToJSON(Solution solution, String output) {
    	/** number of robots */
    	int n=0;
    	if(solution.makespan()==0)
    		System.out.println("Warning: the solution is empty (no robot movements)");
    	else
    		n=solution.steps.get(0).length; // retrieve the number of robots: the size of each array representing robot movements
    	
    	System.out.print("Saving solution to Json file: "+output+" ...");
    	
    	TC.ecritureDansNouveauFichier(output);
    	TC.println("{"); // first line
    	TC.println("  \"instance\": \""+solution.name+"\","); // write instance name
    	
    	int count=0;
    	TC.println("  \"steps\": ["); // start writing all steps
    	for(byte[] k: solution.steps) { // write the movement of robots at the 'k-th' step
    		String line="\t{";
    		for(int i=0;i<n;i++) { // iterate over all robots
    			if(k[i]!=Solution.FIXED) { // the i-th robot is moving
    				char m;
    				if(k[i]==Solution.W) m='W';
    				else if(k[i]==Solution.E) m='E';
    				else if(k[i]==Solution.N) m='N';
    				else if(k[i]==Solution.S) m='S';
    				else throw new Error("Error: movement not recognized");
    				
    				line=line+", \""+i+"\": \""+m+"\"";
    			}
    		}
    		line=line.replaceFirst(", ", ""); // remove ',' at the beginning: for the first robot
    		line=line+"}";
    		if(count!=solution.makespan()-1) // add only for the steps 0, 1, ..., k-2
    			line=line+",";    		
    		
    		TC.println(line); // end encoding for the k-th step
    		count++; // count the number of time steps
    	}
    	TC.println("  ]");
    	
    	// output the width and height
    	TC.println("}");
    	
    	TC.ecritureSortieStandard();
    	System.out.println("done ("+solution.makespan()+" steps)");
    }
	
}
