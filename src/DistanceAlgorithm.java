/**
 * An algorithm that computes a solution of the motion planning problem.
 * 
 * @author Gonzague de Carpentier (Ecole Polytechnique) 
 * @author Dan Meller (Ecole Polytechnique)
 */

import java.util.LinkedList;
import java.util.Arrays;
import java.util.Random;

public class DistanceAlgorithm extends MotionAlgorithm {
	
	/**
	 * CHAMPS ET INITIALISATION --------------------------------------------------------------------------------------------------------
	 */
	
	/** Parameters **/
	int p = 120; 
	int q = 120; 
	int limite = 300; 

	/** An input instance of the motion planning problem **/
	public Instance input;
	
	/** The solution computed by the algorithm **/
	public Solution solution;
	
	/** Current locations of robots **/
	Coordinates current;
	
	/** Number of robots **/
	public int n;
	
	/** Contient la ligne de solution courante **/
	public byte[] currentMoves;
	
	/** Mouvements interdits **/
	public byte[] interdit;
	
	/** Contient les cibles courantes **/
	public Coordinates targets;
	
	/** box[i] Contient les robots qui d�finissent la boite de la cible i  **/
	//box[i][0] contient le robot Nord, 1 : S, 2 : E, 3 :W 
	public int[][] box; 
	
	
	/** Bounding box for grid **/
	public int xmin; 
	public int xmax;
	public int ymin;
	public int ymax;
	
	/** Bounding box for map **/
    public int mxmin;
    public int mxmax;
    public int mymin;
    public int mymax;
    //FIXME : modifier la taille de la map en cours d'éxécution si un robot en sort
	
	/** grid[x][y][i] contient soit -1 si obstacle soit d(i,objectif(i)) **/
	public Integer[][][] grid;
	
	/** map[dx][dy] contient k si le k-ieme robot de current est à la position (mxmin + dx, mymin + dy), -1 s'il y a un obstacle à cette position, et -2 sinon **/
	public int[][] map;
	public int[][] mapCible; 	
	public Random rd = new Random();
	public boolean tousArrives = false;
	public boolean tousEnPlace = false;
	
	public DistanceAlgorithm(Instance input) {
		this.input=input;
		this.solution=new Solution(input.name); // create an empty solution (no steps at the beginning)
		this.current=new Coordinates(this.input.starts.getPositions()); // initialize the current locations with the starting input locations
		n = input.n;
		currentMoves = new byte[n];
		xmin = input.xmin - 1 -p; 
		xmax = input.xmax + 1 + p;
		ymin = input.ymin - 1- p;
		ymax = input.ymax + 1+p;
		mxmin = xmin - 2 * (xmax - xmin);
	    mxmax = xmax + 2 * (xmax - xmin);
	    mymin = ymin - 2 * (ymax - ymin);
	    mymax = ymax + 2 * (ymax - ymin);
	    
	    interdit = new byte[n]; 
	    for (int i=0;i<n;i++) { 
	    	interdit[i]=-1;
	    }
		
		this.initMap();
		this.initMapCible();
		
		targets = new Coordinates(n); 
	    for (int i =0; i<n;i++) { 
	    	targets.setX(i,this.input.targets.getX(i));
	    	targets.setY(i,this.input.targets.getY(i));
	    }
	    
	    initBox(); 
	    
	    // Il faut initialiser les distances aux cibles initiales pour pouvoir trier et faire bouger les cibles
	    this.initGrid();
	    
	    //Dilatation des cibles 
	    moveTargets(p); 
	    
	    this.initGrid();
	}
	
	
	public void initGrid(){
		
		grid = new Integer[xmax - xmin + 1][ymax - ymin + 1][n];
		
	    /** Initialisation des obstacles **/ 
	    for (int j=0; j<this.input.obstacles.size();j++) { 
	    	int x = this.input.obstacles.getX(j);
	    	int y = this.input.obstacles.getY(j); 
	        for (int i=0;i<n;i++) { 
	        	grid[x-xmin][y-ymin][i]=-1;
	        }
	    }
	    /** calcul des distances : parcours en largeur **/ 
		for (int i=0;i<n;i++) { 
		    LinkedList<int[]> Q;
		    Q = new LinkedList<int[]>();
		    int sourceX = targets.getX(i);
		    int sourceY = targets.getY(i);
		    grid[sourceX-xmin][sourceY-ymin][i]=0;
		    Q.add(new int[] {sourceX-xmin,sourceY-ymin}) ; 
			while (!Q.isEmpty()) {
			    int[] caseTablo= Q.pop();
			    int x = caseTablo[0];
			    int y = caseTablo[1]; 
			    
			    for (int dx=-1;dx<=1;dx=dx+2) {
			    	if (x+dx<=xmax-xmin&& x+dx>=0) {
			            if (grid[x+dx][y][i] == null) {
			                grid[x+dx][y][i] = grid[x][y][i]+1;
			                Q.add(new int[] {x +dx,y});
			            }
			    	}
			    } 
		        for (int dy=-1;dy<=1;dy=dy+2) {
		        	if (y+dy<=ymax-ymin&&y+dy>=0) {
				        if (grid[x][y+dy][i] == null) {
				            grid[x][y+dy][i]  = grid[x][y][i]+1;
				            Q.add(new int[] {x,y+dy});
				        }
		        	}
		        }
			}
		}
	}
	
	public void initMap(){
		map = new int[mxmax - mxmin + 1][mymax - mymin + 1];
		int x,y;
	    for (int i = 0; i < map.length; i++){
	        for (int j = 0; j < map[i].length; j++){
	            map[i][j] = -2;
	        }
	    }
	    for (int i = 0; i < input.n; i++){
	        x = input.starts.getX(i);
	        y = input.starts.getY(i);
	        setMap(x,y,i);
		}
		if (this.input.obstacles != null){
		    for (int i = 0; i < input.obstacles.n; i++){
		        x = input.obstacles.getX(i);
		        y = input.obstacles.getY(i);
		        setMap(x,y,-1);
		    }
		}
	}
	
	public void initMapCible(){
		mapCible = new int[mxmax - mxmin + 1][mymax - mymin + 1];
		int x,y;
	    for (int i = 0; i < mapCible.length; i++){
	        for (int j = 0; j < mapCible[i].length; j++){
	            mapCible[i][j] = -2;
	        }
	    }
	    for (int i = 0; i < input.n; i++){
	        x = input.targets.getX(i);
	        y = input.targets.getY(i);
	        setMapCible(x,y,i);
		}
		if (this.input.obstacles != null){
		    for (int i = 0; i < input.obstacles.n; i++){
		        x = input.obstacles.getX(i);
		        y = input.obstacles.getY(i);
		        setMapCible(x,y,-1);
		    }
		}
	}
	
	
	public void initBox() { 
		box = new int[n][8];
		
		
		int[] motion;
		int x,y,j,compteur;
		for (int i=0; i<n;i++) { 
			for (byte mov=1;mov<5;mov++) { 
				motion = byteToMotion(mov);
				x = targets.getX(i)+motion[0];
				y = targets.getY(i)+motion[1];
				j = getMapCible(x,y);
				compteur =0;
				while (j<0 && compteur < p ) { 
					x = x+motion[0];
					y = y+motion[1];
					j = getMapCible(x,y);
					compteur++; 
				}
				if  (j>=0) { 
					box[i][mov-1]=j;
				}
				else { 
					box[i][mov-1]=-2; 
				}
				
				
			}
		}
		
		// On rajoute aussi les axes interm�diaires 
		// 4 : NE 
		// 7 : SW
		// 6 : NW
		// 5 : SE 
		
		int movRef; 
		for (int i=0; i<n;i++) { 
			movRef=4; 
			for (int dx=1;dx>=-1;dx-=2) { 
				for (int dy=1;dy>=-1;dy-=2) { 
					
					x = targets.getX(i)+dx;
					y = targets.getY(i)+dy;
					j = getMapCible(x,y);
					compteur =0;
					while (j<0 && compteur < p ) { 
						x = x+dx;
						y = y+dy;
						j = getMapCible(x,y);
						compteur++; 
					}
					if  (j>=0) { 
						box[i][movRef]=j;
					}
					else { 
						box[i][movRef]=-2; 
					}
					
					movRef+=1;
				}
			}
		}
		
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
		
		while (!tousArrives && solution.steps.size() <= limite) {
			computeOneStep();
			if (solution.steps.size()==q||tousEnPlace) { 
				retourMaison(); 
			}
		}
		System.out.println("Solution computed");
		
	}
	
	/**
	 * Add a new motion step to the current solution
	 */
	public void computeOneStep() {
	    int[] motion = new int[2]; // motion of one robot
	    byte m; //motion of one robot as a byte
	    Integer[] robots = initRobots();  // list of the robots in order of decreasing priority
	    int x, y, i, j;
	    currentMoves = new byte[n];
		for (i = 0; i < n; i++) {
			currentMoves[i] = -1;
		}
	    for (int k = 0; k < robots.length; k++){
	    	i = robots[k];
	    	if (currentMoves[i] == -1) {
		        motion = chooseMotion(i);
		        m = motionToByte(motion);
		        currentMoves[i] = m;
		        if (m != 0) {
			        x = current.getX(i) + motion[0];
			        y = current.getY(i) + motion[1];
			        j = getMap(x,y);
			        while (j >= 0){ // j is a robot who has to move to let i move
			            currentMoves[j] = m;
			            x = x + motion[0];
			            y = y + motion[1];
			            j = getMap(x,y);
			        }
		        }
	    	}
	    }
	    executeMotions();
	}
	
	public Integer[] initRobots() {
		// On trie selon une comparaison des distances inversée (tri descendant) ) 
		Integer[] robots = new Integer[n];
		for (int i = 0; i<n; i++) {
			robots[i] = i;
		}
		Arrays.sort(robots, (Integer a, Integer b) -> Integer.compare(getDistance(b,current.getX(b),current.getY(b)), getDistance(a,current.getX(a),current.getY(a))));
		
		System.out.print("Tour : ");
		System.out.print(solution.makespan());
		System.out.print(" --- Distance Max : ");
		System.out.print(getDistance(robots[0],current.getX(robots[0]),current.getY(robots[0])));
		System.out.print(" --- Atteinte par : ");
		System.out.print(robots[0]);
		System.out.println(" ");
		
		return robots;
	}
	
	public void executeMotions() {
		int[] motion;
		int x, y;
		solution.addStep(currentMoves);
		tousArrives = true;
		tousEnPlace = true; 
		for (int i = 0; i<n;i++) {
			motion = byteToMotion(currentMoves[i]);
			x = current.getX(i);
			y = current.getY(i);
			if (getMap(x,y) == i) {
				setMap(x,y,-2);
			}
			setMap(x + motion[0],y+motion[1], i);
			if (motion[0] == 1) {
				current.increaseX(i);
			}
			if (motion[0] == -1) {
				current.decreaseX(i);
			}
			if (motion[1] == 1) {
				current.increaseY(i);
			}
			if (motion[1] == -1) {
				current.decreaseY(i);
			}
			if (current.getX(i) != input.targets.getX(i) || current.getY(i) != input.targets.getY(i)) {
				tousArrives = false;
			}
			if (current.getX(i) != targets.getX(i) || current.getY(i) != targets.getY(i)) {
				tousEnPlace = false;
			}
		}
	}
	
	public Integer[] checkMove(int i, int[] motion) {
		/**
		* Si le motion est impossible renvoie 0,null
		* Si le motion est OK renvoie 1,contentement 
		* Injonction a la mobilite : 2,axeDeDegagement
		*/
		int x = current.getX(i)+motion[0];
	    int y = current.getY(i)+motion[1];
	    int prioi = getDistance(i,current.getX(i),current.getY(i));
		int j = getMap(x,y);
		int content = 0 ;
		byte bmotion = motionToByte(motion);
		
		if (interdit[i] == bmotion) {
			return new Integer[] {0,null} ;
		}
		
		while(j >=0 && currentMoves[j]==-1) { 
			if (interdit[j]==bmotion) {
				return new Integer[] {0,null}; 
			}
			content += gain(j,motion);
			//On teste si j a la priorité de i - 2 et si oui renvoie null
			//On ne peut pas pousser contre leur gre les robots de meme classe sociale 
			if (getDistance(j,x,y)>=prioi-2) {
				if (gain(j,motion)<0) { 
					return new Integer[] {0,null};
					//Dans ce cas là il s'agit d'une situation de blocage : au prochain tour, j parle avant i et ça oscille 
				}
			}
			x = x+motion[0];
		    y = y+motion[1];
		    j = getMap(x,y);
		}
		
		if (j>=0 && currentMoves[j]==bmotion){
			return new Integer[] {1,content};
		}
		
		//Si la case est vide on verifie que personne ne veuillent venir dessus pour l'instant 
		if (j==-2) {
			for(byte mov = 1;mov<5;mov++) {
				int [] mouv = byteToMotion(mov);
				int k = getMap(x+mouv[0],y+mouv[1]);
				if (k>=0 && currentMoves[k] >= 0) {
					//Le mouvement de la case vide vers k ne doit pas �tre l'oppose de celui de k vers la case vide
					if (rotation180(mov)==currentMoves[k]) { 
						return new Integer[] {0,null};
					}
				}
			}
			return new Integer[] {1,content};
		}
		
		if (j>=0 && currentMoves[j]==0 && gain(j,byteToMotion(rotation180(bmotion)))>0) { 
			int axe = 0 ; 
			if (motion[0]!=0) { axe = 1; } 
			if (motion[1]!=0) { axe = 0; } 
			return new Integer[] {2,axe};
		}
		
		return new Integer[] {0,null}; //S'il y a un robot qui souhaite aller dans une direction perpendiculaire ou un obstacle
	}
	
	
	public int[] chooseMotion(int i) {
		Integer[][] options = new Integer[5][];
		LinkedList<Byte> ameliorations = new LinkedList<Byte>(); // Liste des mouvements qui améliorent la situation du robot et maximisent le contentement des autres
		int m = 0; // m est le maximum des contentements
		byte mtn = 0; //le mouvement que l'on va choisir
		boolean degage = false; 
		int axeDegagement = 0 ; 
		for (byte j = 1; j < 5; j++) {
			options[j] = checkMove(i, byteToMotion(j));
			// MAJ de ameliorations et de m
			if (gain(i, byteToMotion(j)) > 0 && options[j][0] == 1 && (ameliorations.size() == 0 || m <= options[j][1])) {
				if (m < options[j][1]) {
					ameliorations = new LinkedList<Byte>();
				}
				m = options[j][1];
				ameliorations.add(j);
			}
			// MAJ de degage et axeDegagement
			if (options[j][0] == 2) {
				degage = true;
				axeDegagement = options[j][1];
			}
		}
		interdit[i]=-1;
		if (ameliorations.size() == 0 && degage) { // si aucune amélioration n'est possible, on vérifie qu'on ne doit pas dégager
			for (byte sens = 1; sens < 3; sens++) { //FIXME: choisir le meilleur sens de degagement et régler le cas où on ne peut pas dégager perpendiculairement (couloir d'obstacles)
				int j = sens +2*(1-axeDegagement); 
				//On degage perpendiculairement 
				if (options[j][0]==1) { 
					mtn = (byte) j;
					interdit[i]=rotation180(mtn);//Si on doit degager on commence par essayer de remplacer la valeur de mtn par autre chose que 0
				}
			}
		}
		if (ameliorations.size() == 1) { // si une unique amélioration est possible et maximise le contentement on la choisit
			mtn = ameliorations.pop();
		}
		if (ameliorations.size() == 2) { // en cas d'indifférence on choisit au hasard
			if (rd.nextInt(2) == 0){
				mtn = ameliorations.getFirst();
			} else {
				mtn = ameliorations.getLast();
			}
		} //sinon on reste immobile (c'est toujours mieux que de reculer)
		return byteToMotion(mtn);
	}
	
	
	
	public void retourMaison() { 
		//Remet les cibles � leur position initiale 
		
		targets = new Coordinates(n); 
	    for (int i =0; i<n;i++) { 
	    	targets.setX(i,this.input.targets.getX(i)) ;
	    	targets.setY(i,this.input.targets.getY(i)) ;
	    	
	    }
	    
	    this.initGrid();
	    
	}
	
	public int[] calcBarycentre(int i) { 
		//Calcule le barycentre des cibles autour de la cible i (sans renormaliser)
	
		int[] bar = new int[2];
		bar[0]=0;
		bar[1]=0;
		for (int x =-2; x<=2;x++) { 
			for (int y=-2;y<=2;y++) { 
				if (getMapCible(x,y)>=0) { 
					bar[0]+=x;
					bar[1]+=y;
				}
			}
		}
		return bar;
	}
	
	public void moveTarget(int i) { 
		int x = targets.getX(i);
		int y = targets.getY(i);
		int x2,y2;
		LinkedList<Byte> options = new LinkedList<Byte>();
		int[] motion;
		int minProd = 100;
		int prod;
		int[] bary = calcBarycentre(i); 
		boolean N,S,E,W;
		boolean NE,SE,NW,SW;
		
		for (byte j =1 ; j < 5; j++) {
			motion = byteToMotion(j);
			x2 = x +motion[0];
			y2 = y +motion[1]; 
			if (getMapCible(x2,y2)==-2) { 
				//On v�rifie que le mouvement ne sorte pas de la boite : 
				
				E = box[i][2]==-2 || (x2<targets.getX(box[i][2]));
				W = box[i][3]==-2 || (x2>targets.getX(box[i][3]));
				N = box[i][0]==-2 || (y2<targets.getY(box[i][0]));
				S = box[i][1]==-2 || (y2>targets.getY(box[i][1]));
				
				NE= box[i][4]==-2 || (x2<targets.getX(box[i][4])&&y2<targets.getY(box[i][4]));
				SE= box[i][5]==-2 || (x2<targets.getX(box[i][5])&&y2>targets.getY(box[i][5]));
				NW= box[i][6]==-2 || (x2>targets.getX(box[i][6])&&y2<targets.getY(box[i][6]));
				SW= box[i][7]==-2 || (x2>targets.getX(box[i][7])&&y2>targets.getY(box[i][7]));
				
					if (N&&S&&E&&W && NE&&SE&&NW&&SW) { 
					//On cherche � minimiser le produit scalaire avec le barycentre des cibles
					
						prod = motion[0]*bary[0]+motion[1]*bary[1];
						if (prod==minProd) { 
							options.add(j);
						}
						if (prod<minProd) { 
							minProd=prod;
							options = new LinkedList<Byte>();
							options.add(j);
						}
				}
				
		
				
			}
		}
		
		if (options.size()%4==0) { 
			return ;
		}
		
		int k = options.size();
		int choix = rd.nextInt(k);
		byte j = options.get(choix);
		
		motion = byteToMotion(j);
		
		setMapCible(x,y,-2);
		setMapCible(x+motion[0],y+motion[1],i); 
		targets.setX(i, x+motion[0]);
		targets.setY(i, y+motion[1]);
	}
	
	
	public void moveTargets(int p) { 
		Integer[] cibles = new Integer[n];
		int i; 
		for (i=0; i<n;i++) { 
			cibles[i]=i;
		}
		
		for (int tour=0;tour<p;tour++) { 
			Arrays.sort(cibles, (Integer a, Integer b) -> Integer.compare(getDistance(a,targets.getX(a),targets.getY(a)), getDistance(b,targets.getX(b),targets.getY(b))));
			for (int k=0;k<n;k++) { 
				i = cibles[k];
				moveTarget(i);
			}
			
		}
	}
	
	/**
	 * FONCTIONS SECONDAIRES-----------------------------------------------------------------------------------------------------------
	 */
	
	public int getMap(int x, int y) { 
		return map[x-mxmin][y-mymin];
	}
	
	public int getMapCible(int x, int y) { 
		return mapCible[x-mxmin][y-mymin];
	}
	
	public void setMap(int x, int y, int val) { 
		map[x-mxmin][y-mymin]= val;
	}
	
	public void setMapCible(int x, int y, int val) { 
		mapCible[x-mxmin][y-mymin]= val;
	}
	
	
	public int getDistance(int i,int x, int y){
		/**Renvoie la distance du robot i à sa source **/
		/** les deux premiers termes correspondent au retour au rectangle connu**/
		int dRetour= Math.max(Math.max(y-ymax,ymin-y),0)+Math.max(Math.max(x-xmax,xmin-x),0);
		return dRetour + grid[Math.max(Math.min(x, xmax),xmin)-xmin][Math.max(Math.min(y, ymax),ymin)-ymin][i];
	}
	
	public int gain(int i, int[] motion) { 
		//Retourne +1 si jamais le mouv plait � i, -1 si �a lui plait pas et 0 sinon 
		int x = this.current.getX(i);
		int y = this.current.getY(i);
		return getDistance(i,x,y) - getDistance(i,x+motion[0],y+motion[1]);
	}
	
	public void afficheGrid(int i) {
		for (int y = ymax + 10; y >= ymin - 10; y--)  {
			for (int x = xmin - 10; x <= xmax + 10; x++) {
				System.out.print(getDistance(i, x, y));
				System.out.print(" ");
			}
			System.out.println("");
		}
	}
	
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
