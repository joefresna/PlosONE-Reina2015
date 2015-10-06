package nospatial;

import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;

import utils.FileUtils;
import utils.XORShiftRandom;
import nospatial.NSAgent.ActionType;
import nospatial.NSAgent.ControlState;

public class NoSpatial{

	static String DEFAULT_PROPERTIES_FILE = "conf/noSpatial.properties";

	long maxTimeSteps;
	long timeStep;
	List<NSAgent> population;
	long randomSeed;
	String outputFilename;
	String transitionsFilename;
	String interactionsFilename;
	String lastlineFilename;
	String tempFilename = System.getProperty("user.home") + "/tmp/tmpfile.txt";
	int numAgents;
	List<Integer> selectedAgents;
	int numberOfMessageReadInOneTimeStep = 1;
	boolean stopOnConsensus;
	boolean writeTimeEvolutionToFile;
	boolean writeTrasitionTimesToFile;
	boolean writeTempFile = true;
	private double quorum;
	double timeStepSize;
	boolean useOptQualityToComputeDiscovery;
	boolean useOptQualityToComputeAbandonment;
	boolean useOptQualityToComputeRecruitment;
	boolean useOptQualityToComputeCrossinhibition;
	enum RandomDistributions{
		UNIFORM,
		NORMAL,
		TWOGAUSS,
		EXPONENTIAL,
		INVERSE_FRACTION
	};
	RandomDistributions randomDistribution;
	boolean enterIdleState;
	double enterIdleStateProbA;
	double enterIdleStateProbB;
	
	double actionRate;
	double interactionProb;

	XORShiftRandom random;
	BufferedWriter bw, bw2, bw3, bwTmp;
	List<BufferedWriter> bwsRho;
	List<BufferedWriter> bwsSigma;

	List<Transition> linearTransitions;
	List<Transition> nonLinearTransitions;
	int windowSizeNonLinearTrans;

	double gammaA;
	double gammaB;
	double alphaA;
	double alphaB;
	double rhoA;
	double rhoB;
	double sigmaA;
	double sigmaB;
	
	double vMin;
	double vMax;
	double vA;
	double vB;

	int popU;
	int popA;
	int popB;
	Map<String, Integer> previousPopDistribution;
	List<NSAgent> freezedPopulation;

	static public class Transition{
		public String state;
		public String type;
		public long duration;
		
		public Transition(String state, String type, long duration) {
			this.state = state;
			this.type = type;
			this.duration = duration;
		}
	}
	
	static public String GetLetterOfMacrostate(ControlState macroState){
		return (macroState == ControlState.UNCOMMITTED)? "U" :
			(macroState == ControlState.OPTION_A)? "A" : "B";
	}

	public NoSpatial(long seed, String propFilePath) {
		population = new ArrayList<NSAgent>();
		linearTransitions = new ArrayList<Transition>();
		nonLinearTransitions = new ArrayList<Transition>();
		popU = popA = popB = 0;
		selectedAgents = new ArrayList<Integer>();
		previousPopDistribution = new HashMap<String, Integer>();

		ReadPropertiesFile(propFilePath);
		random = new XORShiftRandom(randomSeed);
		ConvertRatesInProbabilities("none");
	}

	void ConvertRatesInProbabilities(String conversionType) {
		if (conversionType.equals("useP0") ){
			double rgammaA = gammaA;
			double rgammaB = gammaB;
			double rrhoA = rhoA;
			double rrhoB = rhoB;
			double ralphaA = alphaA;
			double ralphaB = alphaB;
			double rsigmaA = sigmaA;
			double rsigmaB = sigmaB;
			
			double exitU = rgammaA+rgammaB+rrhoA+rrhoB;
			double exitA = ralphaA + rsigmaB;
			double exitB = ralphaB + rsigmaA;
			gammaA = rgammaA/exitU * (1-Math.exp(-exitU * timeStepSize));
			gammaB = rgammaB/exitU * (1-Math.exp(-exitU * timeStepSize));
			rhoA = rrhoA/exitU * (1-Math.exp(-exitU * timeStepSize));
			rhoB = rrhoB/exitU * (1-Math.exp(-exitU * timeStepSize));
			alphaA = ralphaA/exitA * (1-Math.exp(-exitA * timeStepSize));
			alphaB = ralphaB/exitB * (1-Math.exp(-exitB * timeStepSize));
			sigmaA = rsigmaA/exitB * (1-Math.exp(-exitB * timeStepSize));
			sigmaB = rsigmaB/exitA * (1-Math.exp(-exitA * timeStepSize));
		}
		if (conversionType.equals("straight")){
			gammaA = 1-Math.exp(-gammaA * timeStepSize);
			gammaB = 1-Math.exp(-gammaB * timeStepSize);
			rhoA =   1-Math.exp(-rhoA * timeStepSize);
			rhoB =   1-Math.exp(-rhoB * timeStepSize);
			alphaA = 1-Math.exp(-alphaA * timeStepSize);
			alphaB = 1-Math.exp(-alphaB * timeStepSize);
			sigmaA = 1-Math.exp(-sigmaA * timeStepSize);
			sigmaB = 1-Math.exp(-sigmaB * timeStepSize);
		}
		if ( conversionType.equals("none")){
			gammaA *= timeStepSize;
			gammaB *= timeStepSize;
			rhoA *=   timeStepSize;
			rhoB *=   timeStepSize;
			alphaA *= timeStepSize;
			alphaB *= timeStepSize;
			sigmaA *= timeStepSize;
			sigmaB *= timeStepSize;
		}
	}

	/***********************************/
	/****       INITIALIZATION      ****/
	/***********************************/
	private void start(){
		if (writeTimeEvolutionToFile){
			bw = FileUtils.OpenFileToWrite(outputFilename, "t\tU\tA\tB\n");
		}
		if (writeTrasitionTimesToFile) {
			/** Open the output file writers */
			bw2 = FileUtils.OpenFileToWrite(transitionsFilename, "st\ttr\tt\n");
			bw3 = FileUtils.OpenFileToWrite(interactionsFilename, "st\ttr\tt\n");
			/** Array of output file writes for nonlinear transitions */
			bwsRho = new ArrayList<BufferedWriter>();
			bwsSigma = new ArrayList<BufferedWriter>();
			for (int i = 0; i <= Math.ceil(numAgents / windowSizeNonLinearTrans); i++){
				BufferedWriter bwR = FileUtils.OpenFileToWrite(transitionsFilename + ".R" + i, "st\ttr\tt\n");
				bwsRho.add(bwR);
				BufferedWriter bwS = FileUtils.OpenFileToWrite(transitionsFilename + ".S" + i, "st\ttr\tt\n");
				bwsSigma.add(bwS);
			}
		}
		if (writeTempFile){
			bwTmp = FileUtils.OpenFileToWrite(tempFilename, "IdS\tIdR\tst\ttr\n");
		}

		/** init the population */
		for (int i=0; i < numAgents; i++){
			NSAgent ag = new NSAgent(i);
			
			/* Select from a user-defined distribution the point of switching point of the Step Function */
			double switchingPoint = computeSwitchingPoint();
			if (useOptQualityToComputeRecruitment){
				/* Given the option qualities vA and vB, we compute the RHO transition probabilities using 
				 * a random generated Step Function in [P_vmin,P_vmax] with step at the switching point */
				if (rhoA != rhoB){ System.err.println("With heterogenous case values of rhoA and rhoB must be the same, instead we have rA: " + rhoA + " rB: " + rhoB); }
				double pVMin = computePMinMax(randomDistribution, vMin, rhoA);
				double pVMax = computePMinMax(randomDistribution, vMax, rhoA);
				ag.PrhoA = (vA > switchingPoint)? pVMax : pVMin;
				ag.PrhoB = (vB > switchingPoint)? pVMax : pVMin;
			} else {
				ag.PrhoA = rhoA;
				ag.PrhoB = rhoB;
			}
			if (useOptQualityToComputeDiscovery){
				/* Given the option qualities vA and vB, we compute the GAMMA transition probabilities using 
				 * a random generated Step Function in [P_vmin,P_vmax] with step at the switching point */
				if (gammaA != gammaB){ System.err.println("With heterogenous case values of gammaA and gammaB must be the same, instead we have gA: " + gammaA + " gB: " + gammaB); }
				double pVMin = computePMinMax(randomDistribution, vMin, gammaA);
				double pVMax = computePMinMax(randomDistribution, vMax, gammaA);
				ag.PgammaA = (vA > switchingPoint)? pVMax : pVMin;
				ag.PgammaB = (vB > switchingPoint)? pVMax : pVMin;
			} else {
				ag.PgammaA = gammaA;
				ag.PgammaB = gammaB;
			}
			if (useOptQualityToComputeAbandonment){
				/* Given the option qualities vA and vB, we compute the ALPHA transition probabilities using 
				 * a random generated Step Function in [P_vmin,P_vmax] with step at the switching point */
				double switchingPointAlpha = computeSwitchingPoint(RandomDistributions.INVERSE_FRACTION);
				if (alphaA != alphaA){ System.err.println("With heterogenous case values of alphaA and alphaB must be the same, instead we have aA: " + alphaA + " aB: " + alphaB); }
				double pVMin = computePMinMax(RandomDistributions.INVERSE_FRACTION, vMin, alphaA);
				double pVMax = computePMinMax(RandomDistributions.INVERSE_FRACTION, vMax, alphaA);
				ag.PalphaA = (vA > switchingPointAlpha)? pVMax : pVMin;
				ag.PalphaB = (vB > switchingPointAlpha)? pVMax : pVMin;
			} else {
				ag.PalphaA = alphaA;
				ag.PalphaB = alphaB;
			}
			if (useOptQualityToComputeCrossinhibition){
				/* Given the option qualities vA and vB, we compute the SIGMA transition probabilities using 
				 * a random generated Step Function in [P_vmin,P_vmax] with step at the switching point */
				if (sigmaA != sigmaB){ System.err.println("With heterogenous case values of sigmaA and sigmaB must be the same, instead we have sA: " + sigmaA + " sB: " + sigmaB); }
				double pVMin = computePMinMax(randomDistribution, vMin, sigmaA);
				double pVMax = computePMinMax(randomDistribution, vMax, sigmaA);
				ag.PsigmaA = (vA > switchingPoint)? pVMax : pVMin;
				ag.PsigmaB = (vB > switchingPoint)? pVMax : pVMin;
			} else {
				ag.PsigmaA = sigmaA;
				ag.PsigmaB = sigmaB;
			}
			population.add(ag);
		}
		/* Given the option qualities vA and vB, we compute the enterIdleState-Probabilities */
		enterIdleStateProbA = 1-vA;
		enterIdleStateProbB = 1-vB;

		/** Compute the population distribution */
		popU = popA = popB = 0;
		for (NSAgent agent:population){
			if (agent.currentState == ControlState.OPTION_A){
				popA++;
			}
			if (agent.currentState == ControlState.OPTION_B){
				popB++;
			}
			if (agent.currentState == ControlState.UNCOMMITTED){
				popU++;
			}			
		}
		if (writeTimeEvolutionToFile) {
			WriteStatisticsToFile( bw );
		}
	}

	double computeSwitchingPoint() {
		return computeSwitchingPoint(randomDistribution);
	}
	
	double computeSwitchingPoint(RandomDistributions randomDist) {
		double switchingPoint = -1;
			switch (randomDist){
			case UNIFORM:{
				switchingPoint = random.nextDouble()*(vMax-vMin) + vMin;
				break;
			}
			case NORMAL:
			case TWOGAUSS:
			case EXPONENTIAL:{
				/* NOT SUPPORTED IN THIS VERIONS */
				switchingPoint = 0.5;
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
	
	double computePMinMax(RandomDistributions randomDist, double vMinMax, double rateConstant) {
		double pMinMax = -1;
		switch (randomDist){
		case UNIFORM:{
			pMinMax = rateConstant*vMinMax;
			break;
		}
		case NORMAL:
		case TWOGAUSS:
		case EXPONENTIAL:{
			/* NOT SUPPORTED IN THIS CODE VERSION */
			break;
		}
		case INVERSE_FRACTION:{
			if (vMinMax==0){
				System.err.println("You cannot specify a vMin equal to ZER0 with a f(v)=a/v. Division by zero!!");
			}
			pMinMax = rateConstant/vMinMax;
			break;
		}
		}
		return pMinMax;
	}

	private void freezeState(){
		linearTransitions.clear();
		nonLinearTransitions.clear();
		
		int popUn = (int)Math.floor( (double)popU / windowSizeNonLinearTrans );
		previousPopDistribution.put("popUn", popUn);
		
		int popAn = (int)Math.floor( (double)popA / windowSizeNonLinearTrans );
		previousPopDistribution.put("popAn", popAn);
		
		int popBn = (int)Math.floor( (double)popB / windowSizeNonLinearTrans );
		previousPopDistribution.put("popBn", popBn);
		
		freezedPopulation = cloneList(population);
	}
	
	public static List<NSAgent> cloneList(List<NSAgent> list) {
	    List<NSAgent> clone = new ArrayList<NSAgent>(list.size());
	    for(NSAgent item:list) clone.add(item.clone());
	    return clone;
	}
	
	/***********************************/
	/****     MAIN CONTROL LOOP     ****/
	/***********************************/
	private void step(List<ActionType> actionSequence){
		/** select a random agent */
		int a = random.nextInt(population.size());
		NSAgent agent = population.get(a);
		while (selectedAgents.contains(agent.ID)){
			a = random.nextInt(population.size());
			agent = population.get(a);
		}
		step(actionSequence, a);
	}
	
	private void step(List<ActionType> actionSequence, int agentIndex){
		NSAgent agent = population.get(agentIndex);
		selectedAgents.add(agent.ID);
		
		/** if there is probability of being idle, as fist, we update the Active/Idle state */
		if (enterIdleState){
			agent.updateActiveIdleState(this);
		}

		/** execute transition */
		agent.makeTransitionsAllTogether(this, actionSequence, freezedPopulation);
	}
	
	private void storeTransitions(){
		WriteTransitionsToFile(bw2);
		WriteNonLinearTransitionsToFile(bwsRho, bwsSigma, previousPopDistribution);
		WriteCensoredForPopulationChange(previousPopDistribution);
	}

	/***********************************/
	/****          CLOSING          ****/
	/***********************************/
	private void close(){
		/** Append the final population distribution to the lastline-File */
		if (!FileUtils.AppendToFileAndClose(lastlineFilename, timeStep + "\t" + popU + "\t" + popA + "\t" + popB + "\n"))
			System.err.println("Error in saving the last line to file: " + lastlineFilename + " for experiment with seed: " + randomSeed);
		
		if (writeTimeEvolutionToFile){
			// Close the output file buffer
			try { bw.close(); } 
			catch (IOException e) { e.printStackTrace(); }
		}
		if (writeTrasitionTimesToFile) {
			WriteCensoredDataTransitions(bw2);
			
			// Close the output file buffers
			try { bw2.close(); } 
			catch (IOException e) { e.printStackTrace(); }
			try { bw3.close(); } 
			catch (IOException e) { e.printStackTrace(); }
			try {
				for (int i = 0; i <= Math.ceil(numAgents / windowSizeNonLinearTrans); i++){
					bwsRho.get(i).close();
					bwsSigma.get(i).close();
				}
			}			
			catch (IOException e) { e.printStackTrace(); }
		}
		if (writeTempFile) {
			try { bwTmp.close(); } 
			catch (IOException e) { e.printStackTrace(); }
		}
	}
	
	public static void main(String[] args) {
		/** INIT **/
		String propFile = (args.length > 0)? args[0] : DEFAULT_PROPERTIES_FILE;
		NoSpatial experiment = new NoSpatial(7000, propFile);
		experiment.start();
		
		/** MAIN LOOP **/
		List<ActionType> actionSequence = new ArrayList<NSAgent.ActionType>();
		for (experiment.timeStep = 1; experiment.timeStep <= experiment.maxTimeSteps; experiment.timeStep++){
			experiment.selectedAgents.clear();
			experiment.freezeState();
			
			for (int i = 0; i < experiment.numAgents * experiment.actionRate; i++){
				if (experiment.selectedAgents.size() ==  experiment.population.size()){
					experiment.selectedAgents.clear();
				}
				
				/* define what the agent has to do */
				actionSequence.clear();
				/** Add always the spontaneous */
				actionSequence.add(ActionType.SPONTANEOUS);
				
				/** Flip a "coin" to determine if the interaction is triggered */ 
				if (experiment.interactionProb < 1 && experiment.random.nextDouble() < (1-experiment.interactionProb) ){
					/* VOID IN THIS CODE VERSION */
				} else {
					/** Add the type Interaction when the interactionProb is satisfied  */
					for (int k = 0; k < experiment.numberOfMessageReadInOneTimeStep; k++){
						actionSequence.add(ActionType.INTERACTION);
					}
				}
				/** do the step */
				if (experiment.actionRate == 1){ /* in case actionRate ==1, it is not needed to select randomly an agent -- QUICKER SOLUTION -- */
					experiment.step(actionSequence, i);
				} else {
					experiment.step(actionSequence); /* general case valid for any actionRate */
				}
				
			}
			if (experiment.writeTimeEvolutionToFile) {
				experiment.WriteStatisticsToFile( experiment.bw );
			}
			if (experiment.writeTrasitionTimesToFile) {
				experiment.storeTransitions();
			}
			
			if (experiment.stopOnConsensus && 
					(experiment.popA >= experiment.numAgents*experiment.quorum ||
					 experiment.popB >= experiment.numAgents*experiment.quorum   )){
				break;
			}
		}
		
		/** CLOSE **/
		experiment.close();
	}
	
	private void ReadPropertiesFile(String filename){
		Properties properties = new Properties();
		try {
			properties.load(new FileInputStream(filename));
			
			maxTimeSteps = Long.valueOf( properties.getProperty("timeSteps", "100") );
			randomSeed = Long.valueOf( properties.getProperty("randomSeed", "7000") );
			numAgents = Integer.valueOf( properties.getProperty("numAgents", "10") );
			outputFilename = properties.getProperty("outputFilename", "results/experiment.out");
			transitionsFilename = properties.getProperty("transitionsFilename", "results/tr_noSpatialExp.out");
			interactionsFilename = properties.getProperty("interactionsFilename", "results/int_noSpatialExp.out");
			lastlineFilename = properties.getProperty("lastlineFilename", "results/lastline.txt");
			windowSizeNonLinearTrans = Integer.valueOf( properties.getProperty("windowSizeNonLinearTrans", "10") );
			stopOnConsensus = Boolean.valueOf( properties.getProperty("stopOnConsensus", "false") );
			quorum = Double.valueOf( properties.getProperty("quorum", "1") );
			timeStepSize = Double.valueOf( properties.getProperty("timeStepSize", "1") );
			writeTimeEvolutionToFile = Boolean.valueOf( properties.getProperty("writeTimeEvolutionToFile", "false") );
			writeTrasitionTimesToFile = Boolean.valueOf( properties.getProperty("writeTrasitionTimesToFile", "false") );
			
			actionRate = Double.valueOf( properties.getProperty("actionRate", "1.0") );
			interactionProb = Double.valueOf( properties.getProperty("interactionProb", "0.0") );
			
			gammaA = Double.valueOf( properties.getProperty("gammaA", "0.1") );
			gammaB = Double.valueOf( properties.getProperty("gammaB", "0.1") );
			alphaA = Double.valueOf( properties.getProperty("alphaA", "0.1") );
			alphaB = Double.valueOf( properties.getProperty("alphaB", "0.1") );
			rhoA =   Double.valueOf( properties.getProperty("rhoA",   "0.1") );
			rhoB =   Double.valueOf( properties.getProperty("rhoB",   "0.1") );
			sigmaA = Double.valueOf( properties.getProperty("sigmaA", "0.1") );
			sigmaB = Double.valueOf( properties.getProperty("sigmaB", "0.1") );
			
			vA = Double.valueOf( properties.getProperty("valueA", "1") );
			vB = Double.valueOf( properties.getProperty("valueB", "1") );
			vMin = Double.valueOf( properties.getProperty("vMin", "0") );
			vMax = Double.valueOf( properties.getProperty("vMax", "1") );
			useOptQualityToComputeDiscovery = Boolean.valueOf( properties.getProperty("useOptQualityToComputeDiscovery", "false") );
			useOptQualityToComputeAbandonment = Boolean.valueOf( properties.getProperty("useOptQualityToComputeAbandonment", "false") );
			useOptQualityToComputeRecruitment = Boolean.valueOf( properties.getProperty("useOptQualityToComputeRecruitment", "false") );
			useOptQualityToComputeCrossinhibition = Boolean.valueOf( properties.getProperty("useOptQualityToComputeCrossinhibition", "false") );
			randomDistribution = RandomDistributions.valueOf( properties.getProperty("randomDistribution", "UNIFORM") );
			enterIdleState = Boolean.valueOf( properties.getProperty("enterIdleState", "false") );
			
		} catch (IOException e) {
			System.err.println("Failed to open the properties file: " + filename 
					+ " from current path " + System.getProperty("user.dir") );
			e.printStackTrace();
		}
			
	}

	void WriteStatisticsToFile(BufferedWriter bw) {
		try {
			bw.write(timeStep + "\t" + popU + "\t" + popA + "\t" + popB + "\n" );
		} catch (IOException e) {
			System.err.println("Failed to write stats to file: " + outputFilename);
			e.printStackTrace();
		}
	}
	
	void WriteTransitionsToFile(BufferedWriter bw) {
		// Read all the last transitions that happened in the last time-step
		for (Transition transition:linearTransitions){
			// Write to the file
			try {
				bw.write(transition.state + "\t" + transition.type + "\t" + transition.duration + "\n" );
				bw.flush();
			} catch (IOException e) {
				System.err.println("Failed to write the results in the output file.");
				e.printStackTrace();
			}
		}
	}
	
	/** pops[0] ->> U    pops[1] ->> A    pops[2] ->> B */
	private void WriteNonLinearTransitionsToFile(List<BufferedWriter> bwR, List<BufferedWriter> bwS, Map<String, Integer> pops) {
		// Read all the last non-linear transitions that happened
		for (Transition transition:nonLinearTransitions){
			// Write to the file
			try {
				if (transition.type.equals("IA") || transition.type.equals("CIA") ){
					bwS.get(pops.get("popAn")).write(transition.state + "\t" + transition.type + "\t" + transition.duration + "\n" );
					bwS.get(pops.get("popAn")).flush();
					continue;
				}
				if (transition.type.equals("IB") || transition.type.equals("CIB") ){
					bwS.get(pops.get("popBn")).write(transition.state + "\t" + transition.type + "\t" + transition.duration + "\n" );
					bwS.get(pops.get("popBn")).flush();
					continue;
				}
				if (transition.type.equals("RA") || transition.type.equals("CRA") ){
					bwR.get(pops.get("popAn")).write(transition.state + "\t" + transition.type + "\t" + transition.duration + "\n" );
					bwR.get(pops.get("popAn")).flush();
					continue;
				}
				if (transition.type.equals("RB") || transition.type.equals("CRB") ){
					bwR.get(pops.get("popBn")).write(transition.state + "\t" + transition.type + "\t" + transition.duration + "\n" );
					bwR.get(pops.get("popBn")).flush();
					continue;
				}
			} catch (IOException e) {
				System.err.println("Failed to write the results in the output file.");
				e.printStackTrace();
			}
		}
	}
	
	private void WriteCensoredForPopulationChange(Map<String, Integer> prevPops) {
		/** check if the populations sizes have changed */
		int popAn = (int)Math.floor( (double)popA / windowSizeNonLinearTrans );
		int popBn = (int)Math.floor( (double)popB / windowSizeNonLinearTrans );
		boolean changedA = (popAn != prevPops.get("popAn"));
		boolean changedB = (popBn != prevPops.get("popBn"));
		if (changedA || changedB){
			for (NSAgent ag:population){
				/** REGISTER THE CENSORED DATA */
				switch(ag.currentState){
				/** State U */
				case UNCOMMITTED:{
					if (changedA){ /* population A has changed. Reset all the counters of RECRUITMENT and register censored data */
						long timeSpent = timeStep - ag.timeOfEntranceInCurrentStateWithFixedPopulationA;
						if (timeSpent > 0){
							try {
								bwsRho.get( prevPops.get("popAn") ).write("U\tCRA\t" + timeSpent + "\n");
							} catch (IOException e) {
								System.err.println("Failed to write the results in the output file.");
								e.printStackTrace();
							}
						}
					}
					if (changedB){ /* population B has changed. Reset all the counters of RECRUITMENT and register censored data */
						long timeSpent = timeStep - ag.timeOfEntranceInCurrentStateWithFixedPopulationB;
						if (timeSpent > 0){
							try {
								bwsRho.get( prevPops.get("popBn") ).write("U\tCRB\t" + timeSpent + "\n");
							} catch (IOException e) {
								System.err.println("Failed to write the results in the output file.");
								e.printStackTrace();
							}
						}
					}
					break;
				}
				case OPTION_A:{
					if (changedB){ /* population B has changed. Register censored data for missed interaction */
						long timeSpent = timeStep - ag.timeOfEntranceInCurrentStateWithFixedPopulationB;
						if (timeSpent > 0){
							try {
								bwsSigma.get( prevPops.get("popBn") ).write("A\tCIB\t" + timeSpent + "\n");
							} catch (IOException e) {
								System.err.println("Failed to write the results in the output file.");
								e.printStackTrace();
							}
						}
					}
					break;
				}
				case OPTION_B:{
					if (changedA){ /* population A has changed. Register censored data for missed interaction */
						long timeSpent = timeStep - ag.timeOfEntranceInCurrentStateWithFixedPopulationA;
						if (timeSpent > 0){
							try {
								bwsSigma.get( prevPops.get("popAn") ).write("B\tCIA\t" + timeSpent + "\n");
							} catch (IOException e) {
								System.err.println("Failed to write the results in the output file.");
								e.printStackTrace();
							}
						}
					}
					break;
				}
				}
				if (changedA) ag.timeOfEntranceInCurrentStateWithFixedPopulationA = timeStep;
				if (changedB) ag.timeOfEntranceInCurrentStateWithFixedPopulationB = timeStep;
			}
		}
	}
	
	void WriteCensoredDataTransitions(BufferedWriter bw) {
		// Loop on all the SSParticles and read their status
		for (NSAgent agent:population){
			long timeSpent = timeStep - agent.timeOfEntranceInCurrentState;
			if (timeSpent == 0) continue;
			try{
				switch (agent.currentState){
				case UNCOMMITTED:{
					bw.write("U" + "\t" + "CU" + "\t" + timeSpent + "\n" );
					break;
				}
				case OPTION_A:{
					bw.write("A" + "\t" + "CA" + "\t" + timeSpent + "\n" );
					break;
				}
				case OPTION_B:{
					bw.write("B" + "\t" + "CB" + "\t" + timeSpent + "\n" );
					break;
				}
				}
				bw.flush();
			} catch (IOException e) {
				System.err.println("Failed to write the results in the output file.");
				e.printStackTrace();
			}
		}
	}
	

}
