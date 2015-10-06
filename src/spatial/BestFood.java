package spatial;

import java.awt.Color;
import java.awt.Graphics2D;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;

import utils.Angles;
import utils.FileUtils;
import utils.GenericUtils;
import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.simple.OvalPortrayal2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;
import spatial.BFAgent.ActiveState;
import spatial.BFAgent.CommitmentState;

public class BestFood extends SimState {

	private static final long serialVersionUID = 3772745024873177868L;

	static double MIN_DIFFERENCE_THRESHOLD = 0.00001;
	
	static final String DEFAULT_PROPERTIES_FILE = "conf/spatial.properties";
	
	enum RandomDistributions{
		UNIFORM,
		INVERSE_FRACTION
	};
	
	// Agent's parameters
	public double speed;
	public double localSensing;
	// Agent's vector weights
	public double inertiaWeight;
	public double randomWalkWeight;
	public double logicVectorWeight;
	public double obstacleAvoidanceWeight;
	public double errorVariance;
	public double gammaConstant;
	public double alphaConstant;
	public double rhoConstant;
	public double sigmaConstant;
	public double returnNestProb;
	public double getInactiveProb;
	
	// experiment parameter
	public double timeStepLength;
	public long maxNumStep;
	static public long randomSeed;	
	boolean initialRandomPosition = true;
	boolean initialKnownNestLocation = true;
	public int numAgents;
	public double arenaSize; //diameter
	public double nestSize;
	public double area1size;
	public double area2size;
	// distances of the areas from the center (i.e. the nest)
	public double distanceArea1;
	public double distanceArea2;
	public double angleArea1; 
	public double angleArea2; 
	double qualityArea1;
	double qualityArea2;
	boolean stopOnConsensus;
	double quorum;
	boolean symmetricScenario;
	double vMin;
	double vMax;
	boolean heterogeneousOnDiscovery;
	boolean heterogeneousOnAbandonment;
	boolean heterogeneousOnRecruitment;
	boolean heterogeneousOnCrossinhibition;
	
	// Log file parameters
	String lastlineFilename;
	String outputFilename;
	String transitionsFilename;
	String tempFilename = "results/tempInfo2D.txt";
	boolean writeTimeEvolutionToFile;
	boolean writeTrasitionTimesToFile;
	boolean estimateRatesWithActive;
	boolean writeActiveTransToFile;
	boolean writeTempInfoToFile = false;
	
	// global variables
	private boolean totallyConverged;
	BufferedWriter bw, bw2, bw3, bw4, bw5;
	Map<String, List<BufferedWriter> > bwsMap;
	int nonLinearTransWindowSize;
	List<Integer> populations;
	List<Integer> popWindows;
	List<Integer> prevPopWindows;

	// squared arena
	public Continuous2D arena;
	// circular arena
	public Continuous2D circularArena;
	OvalPortrayal2D oval;
	OvalPortrayal2D nest;
	OvalPortrayal2D area1;
	OvalPortrayal2D area2;
	// position the circular arena in the center of the rectangular arena
	public Double2D arenaPosition;
	public boolean drawVectors = false;
	
	// frozen Scenario params
	boolean frozenScenario;
	int fp_numAgentsStartA;
	int fp_numAgentsStartB;
	enum fp_AnalysisType{ DISCOVERY, ABANDONMENT, RECRUITMENT, CROSSINHIBITION};
	fp_AnalysisType fp_Analysis;
	
	/** GETTERS AND SETTERS */
	/** Vector Weights */
	public double getInertiaWeight() { return inertiaWeight; }
	public void setInertiaWeight(double inertiaWeight) { this.inertiaWeight = inertiaWeight; }
	public double getRandomWalkWeight() { return randomWalkWeight; }
	public void setRandomWalkWeight(double randomWalkWeight) { this.randomWalkWeight = randomWalkWeight; }
	public double getLogicVectorWeight() { return logicVectorWeight; }
	public void setLogicVectorWeight(double logicVectorWeight) { this.logicVectorWeight = logicVectorWeight; }
	
	/** Transition probabilities */
	public double getGammaConstant() { return gammaConstant; }
	public void setGammaConstant(double gammaConstant) { this.gammaConstant = gammaConstant; }
	public double getAlphaConstant() { return alphaConstant; }
	public void setAlphaConstant(double alphaConstant) { this.alphaConstant = alphaConstant; }
	public double getRhoConstant() { return rhoConstant; }
	public void setRhoConstant(double rhoConstant) { this.rhoConstant = rhoConstant; }
	public double getSigmaConstant() { return sigmaConstant; }
	public void setSigmaConstant(double sigmaConstant) { this.sigmaConstant = sigmaConstant; }
	
	/** Agent parameter */
	public double getSpeed() { return speed; }
	public void setSpeed(double speed) { this.speed = speed; }
	public double getReturnNestProb() { return returnNestProb; }
	public void setReturnNestProb(double returnNestProb) { this.returnNestProb = returnNestProb; }
	public double getGetInactiveProb() { return getInactiveProb; }
	public void setGetInactiveProb(double getInactiveProb) { this.getInactiveProb = getInactiveProb; }
	
	/** Scenario parameters */
	public int getNumAgents() { return numAgents; }
	public void setNumAgents(int numAgents) {	this.numAgents = numAgents; }
	
	/** Graphical parameters */
	public boolean isDrawVectors() { return drawVectors; }
	public void setDrawVectors(boolean drawVectors) { this.drawVectors = drawVectors; 	}
	
	/** Statistics */
	public int[] getCommitmentDistribution(){
		Bag agents = arena.getAllObjects();
		int[] distr = new int[agents.numObjs];
		for (int i = 0; i < agents.numObjs; i++){
			distr[i] = ((BFAgent)agents.get(i)).getCommitmentStateInt();
		}
		return distr;
	}

	// create circular object
	public OvalPortrayal2D makeCircularObject(double scale, final int r, final int g, final int b){
		return new OvalPortrayal2D(scale){
			private static final long serialVersionUID = 939961892314196810L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(r, g, b);
				super.draw(object, graphics, info);
			}
			
		};
	}
	
	public BestFood(long seed, String propFilePath) {
		super(seed);
		
		// load the properties file
		ReadPropertiesFile(propFilePath);
		setSeed(randomSeed);
		
		// squared arena
		arena = new Continuous2D(1.0, arenaSize, arenaSize);
		// circular arena
		circularArena = new Continuous2D(1.0, arenaSize, arenaSize);
		oval = makeCircularObject( arenaSize, 200, 200, 200 ); // 124, 230, 228
		nest = makeCircularObject( nestSize, 0, 0, 180 );
		area1 = makeCircularObject( area1size, 255, 0, 0 );
		area2 = makeCircularObject( area2size, 0, 180, 0 );
		// position the circular arena in the center of the rectangular arena
		arenaPosition = new Double2D(arena.getWidth() * 0.5, arena.getHeight() * 0.5);
	}
	
	public void start() {
		super.start();
		
		// clear the arena
		arena.clear();
		
		// positioning circular arena
		circularArena.setObjectLocation(oval, arenaPosition );
		
		// positioning nest	
		Double2D arenaCenter = new Double2D(arena.getWidth() * 0.5, arena.getHeight() * 0.5);
		circularArena.setObjectLocation(nest, arenaCenter);
		
		// positioning area-1
		double xDisplacement = distanceArea1 * Math.cos( Angles.DegreesToRadians(angleArea1) );
		double yDisplacement = distanceArea1 * Math.sin( Angles.DegreesToRadians(angleArea1) );
		circularArena.setObjectLocation(area1, new Double2D(arenaCenter.x + xDisplacement, arenaCenter.y + yDisplacement));

		// positioning Area-2
		xDisplacement = distanceArea2 * Math.cos( Angles.DegreesToRadians(angleArea2) );
		yDisplacement = distanceArea2 * Math.sin( Angles.DegreesToRadians(angleArea2) );
		circularArena.setObjectLocation(area2, new Double2D(arenaCenter.x + xDisplacement, arenaCenter.y + yDisplacement));

		// add some agent to the arena
		Double2D startPosition = new Double2D(arena.getWidth() * 0.5, arena.getHeight() * 0.5);
		for(int i = 0; i < numAgents; i++) 	{
			BFAgent agent = new BFAgent();
			agent.setID(i);
			agent.setTimeStep(0);
			agent.setRandomFoodLocation(this);
			agent.resetCounter(returnNestProb, this);
			/* deploy randomly in the circular arena */
			if (initialRandomPosition){
				double angle = random.nextDouble() * 2 * Math.PI;
				/* Define agent distance from the nest */
				double distance = 0;
				/* Placing the agents in accordance to the asymptotic distribution of active/inactive agents */
				double asyntAct = returnNestProb/(returnNestProb + getInactiveProb);
				int uncommittedPopulation = (frozenScenario)? (numAgents - fp_numAgentsStartA - fp_numAgentsStartB) : numAgents;
				int beginUncommittedIDs = (frozenScenario)? (fp_numAgentsStartA + fp_numAgentsStartB) : 0;
				double nestRadius = nestSize * 0.5;
				if ( (i-beginUncommittedIDs) < asyntAct * uncommittedPopulation ) {
					/* place the agent inside the nest */
					distance = random.nextDouble() * nestRadius;
					/* set the agent as active from the first timestep */
					agent.activeState = ActiveState.ACTIVE;
				} else {
					/* place the agent inside the nest */
					distance = random.nextDouble() * nestRadius;
					/* set the agent as inactive */
					if ( (i-beginUncommittedIDs) > asyntAct * uncommittedPopulation && (i-beginUncommittedIDs) < 2 * asyntAct * uncommittedPopulation) {
						agent.activeState = ActiveState.INACTIVE;
					} else {						
						agent.activeState = ActiveState.PREACTIVE;
					}
				}
				/* if there are frozen populations, we place the committed agents on the path nest-food */
				if (frozenScenario){
					/* agent committed to OPTION_1 */
					if (i < fp_numAgentsStartA){
						/* agent active */
						if ( i < asyntAct * fp_numAgentsStartA ) {
							/* place the agent inside the nest */
							distance = random.nextDouble() * nestRadius;
							/* set the agent as active from the first timestep */
							agent.activeState = ActiveState.ACTIVE;
						}
						/* agent inactive */
						else {
							/* place the agent over the nest-food path */
							distance = random.nextDouble() * (distanceArea1 - nestRadius) + nestRadius;
							angle = Angles.DegreesToRadians(angleArea1);
							/* set the agent as inactive */
							agent.activeState = ActiveState.INACTIVE;
						}
					}
					/* agent committed to OPTION_2 */
					if (i >= fp_numAgentsStartA && i < (fp_numAgentsStartA + fp_numAgentsStartB)){
						/* agent active */
						if ( (i-fp_numAgentsStartA) < asyntAct * fp_numAgentsStartB ) {
							/* place the agent inside the nest */
							distance = random.nextDouble() * nestRadius;
							/* set the agent as active from the first timestep */
							agent.activeState = ActiveState.ACTIVE;
						}
						/* agent inactive */
						else {
							/* place the agent over the nest-food path */
							distance = random.nextDouble() * (distanceArea2 - nestRadius) + nestRadius;
							angle = Angles.DegreesToRadians(angleArea2);
							/* set the agent as inactive */
							agent.activeState = ActiveState.INACTIVE;
						}
					}
				}
				
				/* compute the absolute location-vector from polar coordinates relative to center */
				startPosition = new Double2D( (Math.cos(angle) * distance) + (arena.getWidth() * 0.5), (Math.sin(angle) * distance) + (arena.getHeight() * 0.5) );
			}
			arena.setObjectLocation(agent, startPosition);
			/* set a random initial orientation */
			agent.setOrientation(random.nextDouble() *  Math.PI * 2.0);
			
			/* give partial knowledge to the agents (Food-Location) */
			if (initialKnownNestLocation){
				MutableDouble2D vectorFromAgentToNest = new MutableDouble2D(circularArena.getObjectLocation(nest).subtract(startPosition));
				agent.setVectorFromMeToNest( vectorFromAgentToNest.rotate( -agent.orientation2D()) );
			}
			
			/* set the switching-points for heterogeneous probabilities */
			agent.switchingPointUniform = computeSwitchingPoint(RandomDistributions.UNIFORM);
			agent.switchingPointFraction = computeSwitchingPoint(RandomDistributions.INVERSE_FRACTION);
			
			/* Assign agents 50% to each committed population */
			if (symmetricScenario){
				if (agent.getID() < (numAgents/2)) {
					agent.assignToOption(this, CommitmentState.OPTION_1);
				} else {
					agent.assignToOption(this, CommitmentState.OPTION_2);
				}
			}
			/* Assign frozen populations */
			if (frozenScenario){
				agent.assignAgentToPopulation(this);
			}

			schedule.scheduleRepeating(agent, 1, 1.0);
		}
		
		// initialize the population size array
		populations = new ArrayList<Integer>();
		populations.add(numAgents);
		int NUM_OPTIONS = 2;
		for (int i = 1; i <= NUM_OPTIONS; i++){
			populations.add(0);			
		}
		if (writeTrasitionTimesToFile){
			popWindows = GenericUtils.cloneList(populations);
			for (int i = 0; i < popWindows.size(); i++){
				popWindows.set(i, (int)Math.floor( (double)popWindows.get(i) / nonLinearTransWindowSize ) );
			}
			prevPopWindows = GenericUtils.cloneList(populations);
		}
		
		//Open the output file writer
		if (writeTimeEvolutionToFile) { 
			bw = FileUtils.OpenFileToWrite(outputFilename, "t\tU\tA\tB\n");
		}
		if (writeTrasitionTimesToFile) { 
			bw2 = FileUtils.OpenFileToWrite(transitionsFilename, "t\ttr\n");
			bwsMap = new HashMap<String, List<BufferedWriter>>(4);
			bwsMap.put("RA", new ArrayList<BufferedWriter>());
			bwsMap.put("RB", new ArrayList<BufferedWriter>());
			bwsMap.put("IA", new ArrayList<BufferedWriter>());
			bwsMap.put("IB", new ArrayList<BufferedWriter>());
			for (String key:bwsMap.keySet()) {
				for (int i = 0; i <= Math.ceil(numAgents / nonLinearTransWindowSize); i++){
					BufferedWriter bwtmp = FileUtils.OpenFileToWrite(transitionsFilename + "." + key + i, "t\ttr\n");
					bwsMap.get( key ).add(bwtmp);
				}			    
			}			
		}
		if (writeActiveTransToFile) {
			bw3 = FileUtils.OpenFileToWrite(transitionsFilename + ".act", "t\ttr\tst\n");			
			bw4 = FileUtils.OpenFileToWrite(transitionsFilename + ".act.evo", "t\tU\tA\tB\n");
		}
		if (writeTempInfoToFile) {
			bw5 = FileUtils.OpenFileToWrite(transitionsFilename + ".NST", "t\tU\tA\n");
		}
		
		// initializing the PreControlStepAgent, needed to define the neighbors before the execution of the contolStep
		PreControlStepAgent preCSmanager = new PreControlStepAgent();
		schedule.scheduleRepeating(preCSmanager, 0, 1.0);
		
	}
	
	double computeSwitchingPoint(RandomDistributions randomDist) {
		double switchingPoint = -1;
		switch (randomDist){
		case UNIFORM:{
			switchingPoint = random.nextDouble()*(vMax-vMin) + vMin;
			break;
		}
		case INVERSE_FRACTION:{
			/* Using the Inverse of the CDF of the target random distribution, 
			 * we generate random samples for the target distribution starting from samples generated with uniform distribution */

			/* sample number from uniform distribution */
			double rnd = random.nextDouble();
			/* convert the number to the target probability distribution */
			switchingPoint =  -vMax*vMin /( rnd*(vMax - vMin) -vMax );
			break;
		}
		}
		return switchingPoint;
	}
	
	private void ReadPropertiesFile(String filename){
		Properties properties = new Properties();
		try {
			properties.load(new FileInputStream(filename));
			
			speed = Double.valueOf( properties.getProperty("speed", "0.2") );
			localSensing = Double.valueOf( properties.getProperty("localSensing", "1.0") );

			// Agent's vector weights
			inertiaWeight = Double.valueOf( properties.getProperty("inertiaWeight", "0.01") );
			randomWalkWeight = Double.valueOf( properties.getProperty("randomWalkWeight", "0.01") );
			logicVectorWeight = Double.valueOf( properties.getProperty("logicVectorWeight", "1.0") );
			obstacleAvoidanceWeight = Double.valueOf( properties.getProperty("obstacleAvoidanceWeight", "1.0") );
			errorVariance = Double.valueOf( properties.getProperty("errorVariance", "0.0") );
			gammaConstant = Double.valueOf( properties.getProperty("gammaConstant", "1") );
			alphaConstant = Double.valueOf( properties.getProperty("alphaConstant", "0") );
			rhoConstant = Double.valueOf( properties.getProperty("rhoConstant", "1") );
			sigmaConstant = Double.valueOf( properties.getProperty("sigmaConstant", "1") );
			returnNestProb = Double.valueOf( properties.getProperty("returnNestProb", "0.01") );
			getInactiveProb = Double.valueOf( properties.getProperty("getInactiveProb", "0.01") );

			// Experiment parameter
			timeStepLength = Double.valueOf( properties.getProperty("timeStepLength", "1") );
			randomSeed = Long.valueOf( properties.getProperty("randomSeed", "71285") );
			maxNumStep = Long.valueOf( properties.getProperty("maxNumStep", "0" ) );
			if (maxNumStep == 0) maxNumStep = Long.MAX_VALUE;
			numAgents = Integer.valueOf( properties.getProperty("numAgents", "10") );
			arenaSize = Double.valueOf( properties.getProperty("arenaSize", "100") );
			nestSize = Double.valueOf( properties.getProperty("nestSize", "6") );
			area1size = Double.valueOf( properties.getProperty("area1size", "6") );
			area2size = Double.valueOf( properties.getProperty("area2size", "6") );
			angleArea1 = Double.valueOf( properties.getProperty("angleArea1", "0") );
			distanceArea1 = Double.valueOf( properties.getProperty("distanceArea1", "20") );
			qualityArea1 = Double.valueOf( properties.getProperty("qualityArea1", "0.5") );
			angleArea2 = Double.valueOf( properties.getProperty("angleArea2", "180") );
			distanceArea2 = Double.valueOf( properties.getProperty("distanceArea2", "25") );
			qualityArea2 = Double.valueOf( properties.getProperty("qualityArea2", "0.5") );
			stopOnConsensus = Boolean.valueOf( properties.getProperty("stopOnConsensus", "false") );
			quorum = Double.valueOf( properties.getProperty("quorum", "1") );
			symmetricScenario = Boolean.valueOf( properties.getProperty("symmetricScenario", "false") );
			vMin = Double.valueOf( properties.getProperty("vMin", "0") );
			vMax = Double.valueOf( properties.getProperty("vMax", "1") );
			heterogeneousOnDiscovery = Boolean.valueOf( properties.getProperty("heterogeneousOnDiscovery", "false") );
			heterogeneousOnAbandonment = Boolean.valueOf( properties.getProperty("heterogeneousOnAbandonment", "false") );
			heterogeneousOnRecruitment = Boolean.valueOf( properties.getProperty("heterogeneousOnRecruitment", "false") );
			heterogeneousOnCrossinhibition = Boolean.valueOf( properties.getProperty("heterogeneousOnCrossinhibition", "false") );
			
			outputFilename = properties.getProperty("outputFilename", "results/experiment.out");
			lastlineFilename = properties.getProperty("lastlineFilename", "results/lastline.txt");
			transitionsFilename = properties.getProperty("transitionsFilename", "results/tr_spatialExp.out");
			writeTimeEvolutionToFile = Boolean.valueOf( properties.getProperty("writeTimeEvolutionToFile", "false") );
			writeTrasitionTimesToFile = Boolean.valueOf( properties.getProperty("writeTrasitionTimesToFile", "false") );
			estimateRatesWithActive = Boolean.valueOf( properties.getProperty("estimateRatesWithActive", "true") );
			writeActiveTransToFile = Boolean.valueOf( properties.getProperty("writeActiveTransToFile", "false") );
			nonLinearTransWindowSize = Integer.valueOf( properties.getProperty("nonLinearTransWindowSize", "10") );
			
			frozenScenario = Boolean.valueOf( properties.getProperty("frozenScenario", "false") );
			fp_numAgentsStartA = Integer.valueOf( properties.getProperty("fp_numAgentsStartA", "0") );
			fp_numAgentsStartB = Integer.valueOf( properties.getProperty("fp_numAgentsStartB", "0") );
			fp_Analysis = fp_AnalysisType.valueOf( properties.getProperty("fp_AnalysisType", "RECRUITMENT") );
			
		} catch (IOException e) {
			System.err.println("Failed to open the properties file: " + filename 
					+ " from current path " + System.getProperty("user.dir") );
			e.printStackTrace();
		}
	}
	
	private void WriteStatsToFile(BufferedWriter bw){
		// Write to the file
		try {
			bw.write( Long.toString(schedule.getSteps()) );
			for (Integer pop:populations){
				bw.write("\t" + pop);
			}
			bw.write("\n" );
		} catch (IOException e) {
			System.err.println("Failed to write the results to the output file.");
			e.printStackTrace();
		}
	}
	
	private void WriteFinalCensored() {
		Bag agents = arena.getAllObjects();
		for (int i = 0; i < agents.numObjs; i++){
			BFAgent agent = (BFAgent)agents.get(i);
			try {
				switch (agent.commitmentState) {
				case UNCOMMITTED:{
					if ( fp_Analysis == fp_AnalysisType.DISCOVERY && agent.counterForDiscovery > 0){
						bw2.write(agent.counterForDiscovery + "\t" + "CDA" + "\n");
						bw2.write(agent.counterForDiscovery + "\t" + "CDB" + "\n");
					}
					if ( fp_Analysis == fp_AnalysisType.RECRUITMENT && agent.counterCurrentStateWithFixedPopA > 0){
						bwsMap.get("RA").get( popWindows.get(1) ).write(agent.counterCurrentStateWithFixedPopA + "\tCRA" + "\n");
					}
					if (fp_Analysis == fp_AnalysisType.RECRUITMENT && agent.counterCurrentStateWithFixedPopB > 0){
						bwsMap.get("RB").get( popWindows.get(2) ).write(agent.counterCurrentStateWithFixedPopB + "\tCRB" + "\n");
					}
					break;
				}
				case OPTION_1:{
					if (fp_Analysis == fp_AnalysisType.ABANDONMENT && agent.counterForAlpha > 0){
						bw2.write(agent.counterForAlpha + "\t" + "CLA" + "\n");
					}
					if (fp_Analysis == fp_AnalysisType.CROSSINHIBITION && agent.counterCurrentStateWithFixedPopB > 0){
						bwsMap.get("IB").get( popWindows.get(2) ).write(agent.counterCurrentStateWithFixedPopB + "\tCIB" + "\n");
					}
					break;
				}
				case OPTION_2:{
					if (fp_Analysis == fp_AnalysisType.ABANDONMENT && agent.counterForAlpha > 0){
						bw2.write(agent.counterForAlpha + "\t" + "CLB" + "\n");
					}
					if (fp_Analysis == fp_AnalysisType.CROSSINHIBITION && agent.counterCurrentStateWithFixedPopA > 0){
						bwsMap.get("IA").get( popWindows.get(1) ).write(agent.counterCurrentStateWithFixedPopA + "\tCIA" + "\n");
					}
					break;
				}
				}
			} catch (IOException e) {
				System.err.println("Failed to write to the censored transitions to file " + transitionsFilename);
				e.printStackTrace();
			}

		}
	}
	
	private void UpdatePopulationSizes() {
		// Reset the populations to 0
		for (int i = 0; i < populations.size(); i++){
			populations.set(i, 0);
		}
		
		// Loop on all the agents and increment the population wrt the commitment state
		Bag agents = arena.getAllObjects();
		for (int i = 0; i < agents.numObjs; i++){
			int cmt = ((BFAgent)agents.get(i)).getCommitmentStateInt();
			populations.set(cmt, populations.get(cmt)+1);
		}
		if (writeTrasitionTimesToFile){
			prevPopWindows = GenericUtils.cloneList(popWindows);
			popWindows = GenericUtils.cloneList(populations);
			for (int i = 0; i < popWindows.size(); i++){
				popWindows.set(i, (int)Math.floor( (double)popWindows.get(i) / nonLinearTransWindowSize ) );
			}
		}
	}
	
	private void LogCensoredForPopulationChanges() {	
		/** check if the populations sizes have changed */
		boolean changedA = (!popWindows.get(1).equals( prevPopWindows.get(1) ) );
		boolean changedB = (!popWindows.get(2).equals( prevPopWindows.get(2) ) );
		if (changedA || changedB){
			Bag allAgents = arena.getAllObjects();
			for (int i = 0; i < allAgents.numObjs; i++){
				if (!(allAgents.get(i) instanceof BFAgent) ) continue;
				BFAgent agent =  (BFAgent)allAgents.get(i);
				/** REGISTER THE CENSORED DATA */
				try {
					switch(agent.commitmentState){
					/** State U */
					case UNCOMMITTED:{
						if (fp_Analysis == fp_AnalysisType.RECRUITMENT) {
							if (changedA){ /* population A has changed: register censored data */
								if (agent.counterCurrentStateWithFixedPopA > 0){
									bwsMap.get("RA").get( prevPopWindows.get(1) ).write(agent.counterCurrentStateWithFixedPopA + "\tCRA" + "\n");
								}
							}
							if (changedB){ /* population B has changed: register censored data */
								if (agent.counterCurrentStateWithFixedPopB > 0){
									bwsMap.get("RB").get( prevPopWindows.get(2) ).write(agent.counterCurrentStateWithFixedPopB + "\tCRB" + "\n");
								}
							}
						}
						break;
					}
					/** State A */					
					case OPTION_1:{
						if (fp_Analysis == fp_AnalysisType.CROSSINHIBITION) {
							if (changedB){ /* population B has changed: register censored data */
								if (agent.counterCurrentStateWithFixedPopB > 0){
									bwsMap.get("IB").get( prevPopWindows.get(2) ).write(agent.counterCurrentStateWithFixedPopB + "\tCIB" + "\n");
								}
							}
						}
						break;
					}
					/** State B */ 
					case OPTION_2:{
						if (fp_Analysis == fp_AnalysisType.CROSSINHIBITION) {
							if (changedA){ /* population A has changed: and register censored data */
								if (agent.counterCurrentStateWithFixedPopA > 0){
									bwsMap.get("IA").get( prevPopWindows.get(1) ).write(agent.counterCurrentStateWithFixedPopA + "\tCIA" + "\n");
								}
							}
						}
						break;
					}
					}
				} catch (IOException e) {
					System.err.println("Failed to write the stats in the output file.");
					e.printStackTrace();
				}
				if (changedA) agent.counterCurrentStateWithFixedPopA = 0;
				if (changedB) agent.counterCurrentStateWithFixedPopB = 0;
			}
		}
	}

	public static void main(String[] args)
	{
		String propFile = (args.length > 0)? args[0] : DEFAULT_PROPERTIES_FILE; 
		
		BestFood state = new BestFood( randomSeed, propFile );
		state.nameThread();
		state.start();
		
		// Main control loop
		while(state.schedule.getSteps() < state.maxNumStep && !state.totallyConverged) {
			state.UpdatePopulationSizes();
			if (state.writeTimeEvolutionToFile) state.WriteStatsToFile(state.bw);
			if (!state.schedule.step(state)) break;
			if (state.stopOnConsensus && state.globalDecisionTaken()){
				break;
			}
			if (state.writeTrasitionTimesToFile) state.LogCensoredForPopulationChanges();
			if (state.writeActiveTransToFile) { state.LogActivePopulation(); }
		}
		
		// Terminate the simulation and close buffers
		state.finish();		
		System.exit(0);
	}
	
	@Override
	public void finish() {
		close();
		super.finish();
	}
	
	private void LogActivePopulation() {
		int activeA = 0;
		int activeB = 0;
		int activeU = 0;
		
		// Loop on all the agents and increment the population wrt the commitment state if they are active
		Bag agents = arena.getAllObjects();
		for (int i = 0; i < agents.numObjs; i++){
			BFAgent agent = (BFAgent)agents.get(i);
			if (agent.activeState == ActiveState.ACTIVE){
				switch (agent.commitmentState) {
				case UNCOMMITTED:
					activeU++;
					break;
				case OPTION_1:
					activeA++;
					break;
				case OPTION_2: 
					activeB++;
					break;
				}				
			}
		}
		try {
			bw4.write( Long.toString(schedule.getSteps()) + "\t" + activeU + "\t" + activeA  + "\t" + activeB  + "\n");
		} catch (IOException e) {
			System.err.println("Failed to write the stats in the TMP output file.");
			e.printStackTrace();
		}
	}
	
	private void WriteFinalCensoredActiveInactive() {
		Bag agents = arena.getAllObjects();
		for (int i = 0; i < agents.numObjs; i++){
			BFAgent agent = (BFAgent)agents.get(i);
			String stateSingleLetter = BFAgent.GetSingleLetterForCommitmentState(agent.commitmentState);

			try {
				switch (agent.groundSensorReading) {
				case OVER_NEST:{
					if (agent.counterActive > 0){
						bw3.write(agent.counterActive + "\t" + "CQ" + "\t" + stateSingleLetter + "\n");
					}
					break;
				}
				default:{
					if (agent.counterInactive > 0){
						bw3.write(agent.counterInactive + "\t" + "CE" + "\t" + stateSingleLetter + "\n");
					}
					break;
				}
				}
			} catch (IOException e) {
				System.err.println("Failed to write the stats in the TMP output file.");
				e.printStackTrace();
			}
		}
	}
	
	private boolean globalDecisionTaken() {
		boolean converged = false;
		for (int i=1; i < populations.size(); i++){
			if (populations.get(i) >= numAgents * quorum){
				converged = true;
				break;
			}
		}
		return converged;
	}
	
	private void close() {
		UpdatePopulationSizes();
		if (!FileUtils.AppendToFileAndClose(lastlineFilename, schedule.getSteps() + "\t" + populations.get(0) + "\t" + populations.get(1) + "\t" + populations.get(2) + "\n"))
			System.err.println("Error in saving the last line to file: " + lastlineFilename + " for experiment with seed: " + randomSeed);
		
		// Close the output file buffer
		if (writeTimeEvolutionToFile) {
			WriteStatsToFile(bw);
			try { bw.close(); } 
			catch (IOException e) { e.printStackTrace(); }
		}
		if (writeTrasitionTimesToFile) {
			WriteFinalCensored();
			try { 
				bw2.close();
				for (String key:bwsMap.keySet()){
					for (int i = 0; i <= Math.ceil(numAgents / nonLinearTransWindowSize); i++){
						bwsMap.get(key).get(i).close();
					}				
				}
			} 
			catch (IOException e) { e.printStackTrace(); }
		}
		if (writeActiveTransToFile) {
			WriteFinalCensoredActiveInactive();
			try { 
				bw3.close();
				bw4.close();
			} 
			catch (IOException e) { e.printStackTrace(); }
		}
		if (writeTempInfoToFile) {
			try { bw5.close(); } 
			catch (IOException e) { e.printStackTrace(); }
		}
	}
	
}



