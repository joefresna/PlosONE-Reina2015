package spatial;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.portrayal.Oriented2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;
import spatial.BestFood.RandomDistributions;
import spatial.BestFood.fp_AnalysisType;

public class BFAgent implements Steppable, Oriented2D {
	
	private static final long serialVersionUID = -94988909379815787L;

	public enum CommitmentState {
		UNCOMMITTED,
		OPTION_1,
		OPTION_2
	}
	
	public enum BehaviouralState {
		GO_NEST,
		GO_FOOD,
		RANDOM_WALK
	}
	
	public enum GroundSensorReading { 
		OVER_NOTHING,
		OVER_NEST,
		OVER_FOOD1,
		OVER_FOOD2
	}
	
	public enum ActiveState {
		ACTIVE,
		INACTIVE,
		TRANSITION,
		PREACTIVE
	}
	
	public static String GetSingleLetterForCommitmentState(CommitmentState cs){
		switch (cs) {
		case OPTION_1:
			return "A";
		case OPTION_2:
			return "B";
		default:
			return "U";
		}
	}
	
	public class Message{
		public long agentID;
		public long timeStep;
		public Double2D agentPosition;
		public double agentOrientation;
		public Double2D vectorFromMeToFood;
		public CommitmentState commitmentState;
		public double prho;
		public double psigma;
		public double estimatedQuality;
		
		public Message(long agentID, long timeStep, Double2D agentPosition,
				double agentOrientation, Double2D vectorFromMeToFood, 
				CommitmentState commitmentState, double prho, double psigma,
				double estimatedQuality) {
			super();
			this.agentID = agentID;
			this.timeStep = timeStep;
			this.agentPosition = agentPosition;
			this.agentOrientation = agentOrientation;
			this.vectorFromMeToFood = vectorFromMeToFood;
			this.commitmentState = commitmentState;
			this.prho = prho;
			this.psigma = psigma;
			this.estimatedQuality = estimatedQuality;
		}
		
	}

	// variables
	private long ID; // used for sending an identifier with the messages
	private long timeStep; // used for sending a timeInformation with the messages (necessary for having synchronous communication in single control step loop)
	private double orientation;
	CommitmentState commitmentState;
	BehaviouralState behaviouralState;
	BehaviouralState nextDestination;
	ActiveState activeState;
	boolean justAbandoned;
	boolean triggerActive;
	private int counterLatent;
	private int goBackLimit;
	boolean blockDiscovery;
	GroundSensorReading groundSensorReading;
	private double estimatedQuality;
	private MutableDouble2D obstableAvoidanceVector;
	private MutableDouble2D inertiaVector;
	private MutableDouble2D randomWalkVector;
	private MutableDouble2D logicVector;
	public Double2D myPosition = new Double2D(); // needed only for graphical purposes
	private double previousOrientation = 0;
	private Bag neighborsBag;
	private List<Message> inboxMail;
	private List<Message> inboxMail_future;
	private CommitmentState otherAgentCommitment;
	private double otherAgentTransitionProbability;
	private double otherAgentEstimatedQuality;
	private MutableDouble2D otherAgentVectorToFood; 
	private Double2D otherAgentPosition;
	private double otherAgentOrientation;
	private MutableDouble2D vectorFromMeToFood;
	private MutableDouble2D vectorFromMeToNest;
	
	// transition probabilities
	double pgamma = 0;
	double prho   = 0;
	double palpha = 0;
	double psigma = 0;
	double switchingPointUniform = 0;
	double switchingPointFraction = 0;
	
	// only for debug
	private int neighbors;
	public List<Double2D> convincerPosition = new ArrayList<Double2D>(); // used only for debugging to draw interaction pairs
	public String debugText;
	int counterCurrentState;
	int counterForDiscovery;
	int counterForAlpha;
	int counterCurrentStateWithFixedPopA;
	int counterCurrentStateWithFixedPopB;
	int counterActive;
	int counterInactive;
	
	// getters and setters
	public long getID() { return ID; }
	public void setID(long iD) { ID = iD; }
	public long getTimeStep() { return timeStep; }
	public void setTimeStep(long timeStep) { this.timeStep = timeStep; }
	public Double2D getVectorFromMeToNest() { return new Double2D(vectorFromMeToNest); }
	public Double2D getVectorFromMeToFood() { return new Double2D(vectorFromMeToFood); }
	public void setVectorFromMeToNest(MutableDouble2D vectorFromMeToNest) { this.vectorFromMeToNest = new MutableDouble2D(vectorFromMeToNest); }
	public void setVectorFromMeToFood(MutableDouble2D vectorFromMeToFood) { this.vectorFromMeToFood = new MutableDouble2D(vectorFromMeToFood); }
	public Double2D getObstacleAvoidanceVector() { return new Double2D(obstableAvoidanceVector); }
	public int getCommitmentStateInt() { return commitmentState.ordinal(); }
	public String getCommitmentState() { return commitmentState.name(); }
	public String getBehaviouralState() { return behaviouralState.name(); }
	public String getActiveState() { return activeState.name(); }
	public boolean getJustAbandoned() { return justAbandoned; }
	public boolean getTriggerActive() { return triggerActive; }
	public int getCounterLatent() { return counterLatent; }
	public int getNeighbors() { return neighbors; }
	public void setNeighborsBag(Bag neighborsBag) { this.neighborsBag = neighborsBag; neighbors = this.neighborsBag.size()-1; }
	public void setOrientation(double orientation){ this.orientation = orientation; } 
	public String getDebugText() {	return debugText; }
	
	// constructor
	public BFAgent() {
		ID = 0;
		timeStep = 0;
		orientation = 0;
		commitmentState = CommitmentState.UNCOMMITTED;
		behaviouralState = BehaviouralState.RANDOM_WALK;
		nextDestination = BehaviouralState.GO_NEST;
		activeState = ActiveState.INACTIVE;
		justAbandoned = false;
		triggerActive = false;
		blockDiscovery = false;
		groundSensorReading = GroundSensorReading.OVER_NOTHING;
		vectorFromMeToFood = new MutableDouble2D();
		vectorFromMeToNest = new MutableDouble2D();
		estimatedQuality = 0;
		obstableAvoidanceVector = new MutableDouble2D();
		inertiaVector = new MutableDouble2D();
		randomWalkVector = new MutableDouble2D();
		inboxMail = new ArrayList<Message>();
		inboxMail_future = new ArrayList<Message>();
		resetAllCounters();
	}
	

	@Override
	public double orientation2D() {
		return orientation;
	}

	public double getAngle(Double2D vector){
		return Math.atan2(vector.y, vector.x);
	}
	public double getAngle(MutableDouble2D vector){
		return Math.atan2(vector.y, vector.x);
	}

	@Override
	public void step(SimState arg0) {
		/** 
		 ************************************************** 
		 ********************** INIT ********************** 
		 **************************************************
		*/
		BestFood state = (BestFood) arg0;
		timeStep++;
		myPosition = state.arena.getObjectLocation(this);
		counterCurrentState++;
		
		if (commitmentState != CommitmentState.UNCOMMITTED && blockDiscovery){
			System.out.println("PROBLEM!");
		}
		
		/** 
		 ************************************************** 
		 ***************** SENSOR READING *****************
		 **************************************************
		*/
		/* Check if the agent is inside one of the two 'hot' areas (area1, area2) */
		readGroundSensor(state);
			
		/* Estimate Option Quality */
		CommitmentState opt = 
				(groundSensorReading == GroundSensorReading.OVER_FOOD1)? CommitmentState.OPTION_1 :
				(groundSensorReading == GroundSensorReading.OVER_FOOD2)? CommitmentState.OPTION_2 : CommitmentState.UNCOMMITTED;
		estimateOptionQuality(state, opt);
		
		/** If inside area update the vector with perfect knowledge */
		updateVectorsToTargets_Perfect(state);
		
		/** 
		 ************************************************** 
		 ******************* LOGIC PART *******************
		 **************************************************
		*/
			
		updateActiveInactiveState(state);
		if (state.writeActiveTransToFile){ 
			logActiveInactive(state);
		}
		if (state.writeTrasitionTimesToFile) {
			incrementCountersLogCensored(state);
		}
		
		/** Read incoming messages */
		if (activeState == ActiveState.TRANSITION){
			readMessage(state); 
		}
		clearInboxMail();
		
		updateCommitmentState(state);
		updateBehaviouralState(state);
			
		MutableDouble2D movement = computeMovementVector(state);

		/** 
		 ************************************************** 
		 ***************** ACTUATION EXEC *****************
		 **************************************************
		 */

		// Send messages
		if (activeState == ActiveState.ACTIVE){
			sendMessageToNeighbors(state);
		}
		
		// Perform the agent movement (with collision detection, environmental error, etc.)
		movement = moveAgent(state, movement);
		// this information is used for computing the new position, for graphical reasons and for sending the messages.
		myPosition = state.arena.getObjectLocation(this);
		
		// Update the vectors to the target areas 
		// [THE UPDATE SHOULD BE DONE AT THE BEGINNING OF THE NEXT TIMESTEP, BUT WE PUT IT HERE FOR GRAPHICAL/DEBUG REASONS]
		updateVectorsToTargets( movement );
		
	}
	
	private void updateActiveInactiveState(BestFood state) {
		if (activeState == ActiveState.INACTIVE || activeState == ActiveState.PREACTIVE) {
			counterLatent--;
			if (counterLatent <= 0){
				triggerActive = true;
			}
		} else {
			triggerActive = false;
		}
		
		switch (activeState) {
		case ACTIVE:{
			/* With probability getInactiveProb, the agent gets INACTIVE spontaneously */
			double rnd = state.random.nextDouble();
			if (rnd < state.getInactiveProb){
				activeState = ActiveState.TRANSITION;
			}
			break;
		}
		/* if INACTIVE, we check if gets active */
		case INACTIVE:{
			/* First precondition to turn active is to enter the NEST and to have the triggerActive */
			if ( groundSensorReading == GroundSensorReading.OVER_NEST){
				switch (commitmentState) {
				/* if UNCOMMITTED */
				case UNCOMMITTED:{
					/* Get active only if for triggerActive true */
					if (behaviouralState == BehaviouralState.GO_NEST) {
						if (triggerActive) {
							activeState = ActiveState.ACTIVE;
							triggerActive = false;
						} else {
							if (justAbandoned){
								activeState = ActiveState.INACTIVE;								
							} else {
								activeState = ActiveState.PREACTIVE;
							}
						}
					}
					break;
				}
				case OPTION_1:
				case OPTION_2:{
					/** just entered && not going to perform other transitions */
					if (nextDestination == BehaviouralState.GO_NEST) {
						if (triggerActive) {
							activeState = ActiveState.ACTIVE;
							triggerActive = false;
						} else {
							activeState = ActiveState.PREACTIVE;
						}
					}
					break;
				}
				}
			}
			break;
		}
		/* if the agents is PREACTIVE, it switches as soon the triggerActive flips */
		case PREACTIVE:{
			if (triggerActive){
				activeState = ActiveState.ACTIVE;
				triggerActive = false;
			}
			break;
		}
		/* if in TRANSITION, the agents switch immediately to INACTIVE state */
		case TRANSITION:{
			/* the agents stays only one state in TRANSITION and turn INACTIVE */
			activeState = ActiveState.INACTIVE;
			/* if the agent is UNCOMMITTED, when he turns INACTIVE, he select a random direction of exploration */
			if (commitmentState == CommitmentState.UNCOMMITTED){
				setRandomFoodLocation(state);
			}
			resetCounter(state.returnNestProb, state);
			break;
		}
		}
	}
	
	/* Reset the counterLatent with a random value from an exponential distribution to resemble a memoryless process */
	void resetCounter(double PLatent, BestFood state) {
		double rnd = state.random.nextDouble();
		counterLatent = (int)Math.round( Math.log(rnd)/(-PLatent) );
		goBackLimit = counterLatent/2;
	}
	
	void setRandomFoodLocation(BestFood state){
		double maxLength = 50;
		double randomAngle = state.random.nextDouble() * Math.PI * 2;
		double xval = Math.cos(randomAngle) * maxLength;
		double yval = Math.sin(randomAngle) * maxLength;
		vectorFromMeToFood.setTo(xval, yval);
		nextDestination = BehaviouralState.GO_FOOD;
	}
	
	private void incrementCountersLogCensored(BestFood state) {
		if (state.estimateRatesWithActive){
			/** Log censored */
			if (activeState == ActiveState.INACTIVE || activeState == ActiveState.PREACTIVE){
				if (counterCurrentStateWithFixedPopA > 0){
					switch (commitmentState) {
					case UNCOMMITTED:
						if (state.fp_Analysis == fp_AnalysisType.RECRUITMENT)
							writeTransitionLog(state, "CRA");									
						break;
					case OPTION_1:
						break;
					case OPTION_2:
						if (state.fp_Analysis == fp_AnalysisType.CROSSINHIBITION)
							writeTransitionLog(state, "CIA");									
						break;
					}
				}
				if (counterCurrentStateWithFixedPopB > 0){
					switch (commitmentState) {
					case UNCOMMITTED:
						if (state.fp_Analysis == fp_AnalysisType.RECRUITMENT)
							writeTransitionLog(state, "CRB");									
						break;
					case OPTION_1:
						if (state.fp_Analysis == fp_AnalysisType.CROSSINHIBITION)
							writeTransitionLog(state, "CIB");									
						break;
					case OPTION_2:
						break;
					}
				}
			}
			
			/** update the counters for RECRUITMENT AND CROSS-INHIBITON */
			if (activeState == ActiveState.ACTIVE || activeState == ActiveState.TRANSITION){
				counterCurrentStateWithFixedPopA++;
				counterCurrentStateWithFixedPopB++;
			} else {
				counterCurrentStateWithFixedPopA = 0;
				counterCurrentStateWithFixedPopB = 0;
			}
			
			/** COUNTER FOR DISCOVERY */
			if ((activeState == ActiveState.INACTIVE || activeState == ActiveState.PREACTIVE) && commitmentState == CommitmentState.UNCOMMITTED){
				counterForDiscovery++;
			}
			else {
				if (state.fp_Analysis == fp_AnalysisType.DISCOVERY && counterForDiscovery > 0){
					writeTransitionLog(state, "CDA+CDB");
				}
				counterForDiscovery = 0;
			}
			
			/** COUNTER FOR ALPHA */
			if ((activeState == ActiveState.INACTIVE || activeState == ActiveState.PREACTIVE) && commitmentState != CommitmentState.UNCOMMITTED){
				counterForAlpha++;
			}
			else {
				if (state.fp_Analysis == fp_AnalysisType.ABANDONMENT && counterForAlpha > 0){
					if (commitmentState == CommitmentState.OPTION_1) {
						writeTransitionLog(state, "CLA");
					}
					if (commitmentState == CommitmentState.OPTION_2) {
						writeTransitionLog(state, "CLB");
					}
				}
				counterForAlpha = 0;
			}
		
		}
		else {
			counterCurrentStateWithFixedPopA++;
			counterCurrentStateWithFixedPopB++;
			if (commitmentState == CommitmentState.UNCOMMITTED){
				counterForDiscovery++;
			} else {
				counterForDiscovery = 0;
			}
			if (commitmentState != CommitmentState.UNCOMMITTED){
				counterForAlpha++;
			} else {
				counterForAlpha = 0;
			}
		}
	}
	
	private void logActiveInactive(BestFood state) {
		try {
			switch (activeState) {
			case ACTIVE:{
				counterActive++;
				if (counterInactive > 0){
					state.bw3.write(counterInactive + "\t" + "E" + "\t" + GetSingleLetterForCommitmentState(commitmentState) + "\n");
				}
				counterInactive = 0;
				break;
			}
			case INACTIVE:
			case PREACTIVE:{
				counterInactive++;
				counterActive = 0;				
				break;
			}
			case TRANSITION:{
				state.bw3.write(counterActive + "\t" + "Q" + "\t" + GetSingleLetterForCommitmentState(commitmentState) + "\n");
				counterActive = 0;
				break;
			}
			}				
		} catch (IOException e) {
			System.err.println("Failed to write to log info into the ActiveTrans file.");
			e.printStackTrace();
		}
	}
	
	private MutableDouble2D computeMovementVector(BestFood state) {
		// Calculate Inertia Vector
		inertiaVector = new MutableDouble2D( Math.cos(orientation), Math.sin(orientation) );

		// Calculate Random-Walk Vector
		randomWalkVector = uncorrelatedRandomWalk(state);
		
		switch (behaviouralState) {
		case RANDOM_WALK:{
			// Set to Zero the Logic-Vector
			logicVector = new MutableDouble2D(0,0);
			break;
		}
		case GO_NEST:{
			// Calculate Logic-Vector towards Nest
			logicVector = followTheVector(vectorFromMeToNest);
			break;
		}
		case GO_FOOD:{
			// Calculate Logic-Vector towards Food
			logicVector = followTheVector(vectorFromMeToFood);			
			break;
		}
		}

		// Multiply the vectors by their weights
		inertiaVector.multiplyIn(state.inertiaWeight);
		randomWalkVector.multiplyIn(state.randomWalkWeight);
		logicVector.multiplyIn(state.logicVectorWeight);
		
		// Sum up all the vectors
		MutableDouble2D movement = new MutableDouble2D(0,0);
		movement.addIn(inertiaVector);
		movement.addIn(randomWalkVector);
		movement.addIn(logicVector);
		
		return movement;
	}
	
	private void updateBehaviouralState(BestFood state) {
		/** an UNCOMMITTED agent arrived to the NEST after abandonment (justAbandoned), reset the variable */
		/** an UNCOMMITTED agent arrived to the NEST after refusing discovery (blockDiscovery), reset the variable */
		if (groundSensorReading == GroundSensorReading.OVER_NEST){
			if (justAbandoned ){
				setRandomFoodLocation(state);
				justAbandoned = false;
			}
			if (blockDiscovery) {
				blockDiscovery = false;
			}
		} 
		
		/* if the agent is ACTIVE, it random walks inside the nest */
		if (activeState == ActiveState.ACTIVE || activeState == ActiveState.PREACTIVE) {
			behaviouralState = BehaviouralState.RANDOM_WALK;
		}
		/* if INACTIVE or TRANSITION, depending on the state and the location switches behaviour */
		else {
			switch (commitmentState) {
			case UNCOMMITTED:{
				/** if the agent has decided to get ACTIVE or after abandonment, it goes straight to the NEST */
				if (triggerActive || justAbandoned || blockDiscovery || counterLatent <= goBackLimit) {
					behaviouralState = BehaviouralState.GO_NEST;
				}
				/** otherwise, the agent does RANDOM WALK */
				else {
					double stepLengthSq = (state.speed * state.timeStepLength) * (state.speed * state.timeStepLength);
					if ( vectorFromMeToFood.lengthSq() <= stepLengthSq) {
						nextDestination = BehaviouralState.GO_NEST;
					}
					behaviouralState = nextDestination;
				}
				break;
			}
			case OPTION_1:
			case OPTION_2:{
				switch (groundSensorReading) {
				case OVER_NEST:{
					/* A committed agent in the nest always performs RANDOM WALK */
					behaviouralState = BehaviouralState.RANDOM_WALK;
					nextDestination = (counterLatent > goBackLimit)? BehaviouralState.GO_FOOD : BehaviouralState.GO_NEST;
					break;
				}
				case OVER_FOOD1:
				case OVER_FOOD2:{
					/* A committed agent in the food always performs RANDOM WALK */
					if (triggerActive || counterLatent <= goBackLimit){
						behaviouralState = BehaviouralState.GO_NEST;
					} else {
						behaviouralState = BehaviouralState.RANDOM_WALK;
					}
					nextDestination = BehaviouralState.GO_NEST;
					break;
				}
				case OVER_NOTHING:{
					if (triggerActive || counterLatent <= goBackLimit){
						nextDestination = BehaviouralState.GO_NEST;
					}
					behaviouralState = nextDestination;
					break;
				}
				}
				break;
			}
			}
		}
	}
	
	private MutableDouble2D followTheVector(MutableDouble2D vectorToFollow) {
		// Clone the vector
		MutableDouble2D resultVector = new MutableDouble2D( vectorToFollow );
		// (i) Rotate it in the simulator reference system
		resultVector.rotate(orientation);
		// (ii) Resize to unit size 
		resultVector.resize(1.0);
		
		return resultVector;
	}
	
	@SuppressWarnings("unused")
	private void writeDataForEstimatingWellMixed(BestFood state) {
		double Us=0, As=0, Bs=0;
		if (!inboxMail.isEmpty()){
			for (Message mex:inboxMail){
				switch (mex.commitmentState) {
				case UNCOMMITTED:{
					Us++;
					break;
				}
				case OPTION_1:{
					As++;
					break;
				}
				case OPTION_2:{
					Bs++;
					break;
				}
				}
			}
			Us /= inboxMail.size();
			As /= inboxMail.size();
			Bs /= inboxMail.size();
		}
		try {
			state.bw3.write((timeStep-1) + "\t" + ID + "\t" + Us + "\t" + As + "\t" + Bs + "\n");
		} catch (IOException e) {
			System.err.println("Failed to write to the transitions file at timestep " + timeStep + ".");
			e.printStackTrace();
		}
	}
	
	private double computePMinMax(RandomDistributions randomDist, double vMinMax, double rateConstant) {
		double pMinMax = -1;
		switch (randomDist){
		case UNIFORM:{
			pMinMax = rateConstant*vMinMax;
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
	
	private double functionForPGamma(double estQuality, BestFood state) {
		double pg;
		if (state.heterogeneousOnDiscovery){
			double pVMin = computePMinMax(RandomDistributions.UNIFORM, state.vMin, state.gammaConstant);
			double pVMax = computePMinMax(RandomDistributions.UNIFORM, state.vMax, state.gammaConstant);
			pg = (estQuality > switchingPointUniform)? pVMax : pVMin;
		} else {			
			pg = estQuality * state.gammaConstant;
		}
		return pg;
	}
	
	private double functionForPAlpha(double estQuality, BestFood state) {
		double pa;
		if (state.heterogeneousOnAbandonment){
			double pVMin = computePMinMax(RandomDistributions.INVERSE_FRACTION, state.vMin, state.alphaConstant);
			double pVMax = computePMinMax(RandomDistributions.INVERSE_FRACTION, state.vMax, state.alphaConstant);
			pa = (estQuality > switchingPointFraction)? pVMax : pVMin;
		} else {
			pa = state.alphaConstant / estQuality;
		}
		return pa;
	}
	
	private double functionForPRho(double estQuality, BestFood state) {
		double pr;
		if (state.heterogeneousOnRecruitment){
			double pVMin = computePMinMax(RandomDistributions.UNIFORM, state.vMin, state.rhoConstant);
			double pVMax = computePMinMax(RandomDistributions.UNIFORM, state.vMax, state.rhoConstant);
			pr = (estQuality > switchingPointUniform)? pVMax : pVMin;
		} else {			
			pr = estQuality * state.rhoConstant;
		}
		return pr;
	}
	
	private double functionForPSigma(double estQuality, BestFood state) {
		double ps;
		if (state.heterogeneousOnCrossinhibition){
			double pVMin = computePMinMax(RandomDistributions.UNIFORM, state.vMin, state.sigmaConstant);
			double pVMax = computePMinMax(RandomDistributions.UNIFORM, state.vMax, state.sigmaConstant);
			ps = (estQuality > switchingPointUniform)? pVMax : pVMin;
		} else {			
			ps = estQuality * state.sigmaConstant;
		}
		return ps;
	}
	
	private void updateCommitmentState(BestFood state) {
		double rnd = state.random.nextDouble();

		switch (activeState) {
		case TRANSITION:{
			switch (commitmentState) {
			case UNCOMMITTED:{
				/* RECRUITMENT */
				/** only if I've interacted with a committed agent at this time step, I can get recruited with prho > 0*/
				prho = (otherAgentCommitment != null && otherAgentCommitment != CommitmentState.UNCOMMITTED)? otherAgentTransitionProbability : 0;
				if (rnd < prho){
					/** write transition data for survival analysis */
					if (state.writeTrasitionTimesToFile) {
						if (state.fp_Analysis == fp_AnalysisType.RECRUITMENT){
							switch (otherAgentCommitment) {
							case OPTION_1:{
								writeTransitionLog(state, "RA");
								break;
							}
							case OPTION_2:{
								writeTransitionLog(state, "RB");
								break;
							}
							default:{break;}
							}
							resetAllCounters();
						}
					}
					
					/** perform commitment transition */
					if (!state.frozenScenario){
						commitmentState = otherAgentCommitment;
						Double2D vectorFromMeToHim = otherAgentPosition.subtract(myPosition);
						vectorFromMeToFood = translateHisVectorInMyRefFrame(otherAgentVectorToFood, otherAgentOrientation, vectorFromMeToHim);
						estimatedQuality = otherAgentEstimatedQuality;
						behaviouralState = BehaviouralState.GO_FOOD; // makes a recruited agent stop RANDOM_WALK in nest and go directly to NEST
						nextDestination = BehaviouralState.GO_FOOD;
					}
					
				}
				break; // break for UNCOMMITTED --inside at active--
			}
			case OPTION_1:{
				/* CROSS-INHIBITION */
				/** only if the agent interacted with another agent committed to OPT_2, we have psigma > 0 */
				psigma = (otherAgentCommitment != null && otherAgentCommitment == CommitmentState.OPTION_2)? otherAgentTransitionProbability : 0;
				if (rnd < psigma){
					/** write transition data for survival analysis */
					if (state.writeTrasitionTimesToFile) {
						if (state.fp_Analysis == fp_AnalysisType.CROSSINHIBITION){
							writeTransitionLog(state, "IB");				
							resetAllCounters();
						}
					}
					
					/** perform commitment transition */
					if (!state.frozenScenario){
						commitmentState = CommitmentState.UNCOMMITTED;
						behaviouralState = BehaviouralState.RANDOM_WALK;
						estimatedQuality = 0;
						setRandomFoodLocation(state);
					}
					
				}
				break; // break for OPTION1 --inside at active--
			}
			case OPTION_2:{
				/* CROSS-INHIBITION */
				/** only if the agent interacted with another agent committed to OPT_1, we have psigma > 0 */
				psigma = (otherAgentCommitment != null && otherAgentCommitment == CommitmentState.OPTION_1)? otherAgentTransitionProbability : 0;
				if (rnd < psigma){
					/** write transition data for survival analysis */
					if (state.writeTrasitionTimesToFile) {
						if (state.fp_Analysis == fp_AnalysisType.CROSSINHIBITION){
							writeTransitionLog(state, "IA");
							resetAllCounters();
						}
					}
					
					/** perform commitment transition */ 
					if (!state.frozenScenario){
						commitmentState = CommitmentState.UNCOMMITTED;
						behaviouralState = BehaviouralState.RANDOM_WALK;
						estimatedQuality = 0;
						setRandomFoodLocation(state);
					}
					
				}
				break; // break for OPTION2 --inside at active--
			}
			}
			break; // break for activeState:TRANSITION
		}
		case PREACTIVE:
		case INACTIVE:{
			switch (commitmentState) {
			case UNCOMMITTED:{
				/* DISCOVERY */
				if ((groundSensorReading == GroundSensorReading.OVER_FOOD1 || groundSensorReading == GroundSensorReading.OVER_FOOD2) && !justAbandoned && !blockDiscovery){
					pgamma = functionForPGamma(estimatedQuality, state);
					if (rnd < pgamma) {
						behaviouralState = BehaviouralState.GO_NEST;
						nextDestination = BehaviouralState.GO_NEST;

						/** write transition data for survival analysis */
						if (state.writeTrasitionTimesToFile) {
							if (state.fp_Analysis == fp_AnalysisType.DISCOVERY){
								if (groundSensorReading == GroundSensorReading.OVER_FOOD1) {
									writeTransitionLog(state, "DA");
								}
								if (groundSensorReading == GroundSensorReading.OVER_FOOD2) {
									writeTransitionLog(state, "DB");
								}
								resetAllCounters();
								resetCounter(state.returnNestProb, state);
								/* teletransportation in a random point within the nest area  */
								double angle = state.random.nextDouble() * 2 * Math.PI;
								double distance = state.random.nextDouble() * state.nestSize * 0.5;
								state.arena.setObjectLocation(this, new Double2D( (Math.cos(angle) * distance) + (state.arena.getWidth() * 0.5), (Math.sin(angle) * distance) + (state.arena.getHeight() * 0.5) ));
								myPosition = state.arena.getObjectLocation(this);
								setRandomFoodLocation(state);
							}
						}
						/** perform commitment transition */
						if (!state.frozenScenario){
							if (groundSensorReading == GroundSensorReading.OVER_FOOD1) {
								commitmentState = CommitmentState.OPTION_1;
							}
							if (groundSensorReading == GroundSensorReading.OVER_FOOD2) {							
								commitmentState = CommitmentState.OPTION_2;
							}
						}	
					}
					/* if the discovery probability didn't trigger */
					else {
						estimatedQuality = 0;
						/* the agent decided to do not commit therefore blocks further discovery */
						blockDiscovery = true;
					}
				}
				
				break; // break for UNCOMMITTED --inside at inactive--
			}
			case OPTION_1:
			case OPTION_2:{
				/* ABANDONMENT */
				palpha = functionForPAlpha(estimatedQuality, state);
				if (rnd < palpha){
					
					/** write transition data for survival analysis */
					if (state.writeTrasitionTimesToFile) {
						if (state.fp_Analysis == fp_AnalysisType.ABANDONMENT){
							if (commitmentState == CommitmentState.OPTION_1) {
								writeTransitionLog(state, "LA");
							}
							if (commitmentState == CommitmentState.OPTION_2) {
								writeTransitionLog(state, "LB");
							}
							resetAllCounters();
						}
					}
					
					/** perform abandonment transition */
					if (!state.frozenScenario){
						commitmentState = CommitmentState.UNCOMMITTED;
						vectorFromMeToFood = new MutableDouble2D(vectorFromMeToNest);
						estimatedQuality = 0;
						/** when the abandonment transition triggers, the agent goes back to the nest */
						justAbandoned = true;
						behaviouralState = BehaviouralState.GO_NEST;
						nextDestination= BehaviouralState.GO_NEST;
						/** I force to get INACTIVE because I could be PREACTIVE */
						activeState = ActiveState.INACTIVE;
					}
					
				}
				break; // break for OPTIONS --inside at inactive--
			}
			}
			break; // break for INACTIVE and PREACTIVE
		}
		case ACTIVE:{
			/* Do nothing ! */
			break;
		}
		}
	}

	
	private void writeTransitionLog(BestFood state, String transition){
		writeTransitionLog(state, transition, "" );
	}
	
	private void writeTransitionLog(BestFood state, String transition, String commState){
		try {
			if (transition.equals("DA")){
				state.bw2.write(counterForDiscovery + "\t" + "DA" + "\n");
				if (!state.frozenScenario){
					// censored recruitments
					if (counterCurrentStateWithFixedPopA > 0)
						state.bwsMap.get("RA").get(state.popWindows.get( CommitmentState.OPTION_1.ordinal() )).write( counterCurrentStateWithFixedPopA + "\t" + "CRA" + "\n" );
					if (counterCurrentStateWithFixedPopB > 0)
						state.bwsMap.get("RB").get(state.popWindows.get( CommitmentState.OPTION_2.ordinal() )).write( counterCurrentStateWithFixedPopB + "\t" + "CRB" + "\n" );
				}
				return;
			}
			if (transition.equals("DB")){
				state.bw2.write(counterForDiscovery + "\t" + "DB" + "\n");
				state.bw2.write(counterForDiscovery + "\t" + "CDA" + "\n");
				if (!state.frozenScenario){
					// censored recruitments
					if (counterCurrentStateWithFixedPopA > 0)
						state.bwsMap.get("RA").get(state.popWindows.get( CommitmentState.OPTION_1.ordinal() )).write( counterCurrentStateWithFixedPopA + "\t" + "CRA" + "\n" );
					if (counterCurrentStateWithFixedPopB > 0)
						state.bwsMap.get("RB").get(state.popWindows.get( CommitmentState.OPTION_2.ordinal() )).write( counterCurrentStateWithFixedPopB + "\t" + "CRB" + "\n" );
				}
				return;
			}
			if (transition.equals("CDA+CDB")){
				state.bw2.write(counterForDiscovery + "\t" + "CDA" + "\n");
				return;
			}
			if (transition.equals("LA")){
				state.bw2.write(counterForAlpha + "\t" + "LA" + "\n");
				if (!state.frozenScenario){
					// censored cross-inhibition
					if (counterCurrentStateWithFixedPopB > 0)
						state.bwsMap.get("IB").get(state.popWindows.get(CommitmentState.OPTION_2.ordinal())).write( counterCurrentStateWithFixedPopB + "\t" + "CIB" + "\n" );
				}
				return;
			}
			if (transition.equals("LB")){
				state.bw2.write(counterForAlpha + "\t" + "LB" + "\n");
				if (!state.frozenScenario){
					// censored cross-inhibition
					if (counterCurrentStateWithFixedPopA > 0)
						state.bwsMap.get("IA").get(state.popWindows.get(CommitmentState.OPTION_1.ordinal())).write( counterCurrentStateWithFixedPopA + "\t" + "CIA" + "\n" );
				}
				return;
			}
			if (transition.equals("CLA")){
				state.bw2.write(counterForAlpha + "\t" + "CLA" + "\n");
				return;
			}
			if (transition.equals("CLB")){
				state.bw2.write(counterForAlpha + "\t" + "CLB" + "\n");
				return;
			}
			if (transition.equals("RA")){
				state.bwsMap.get("RA").get(state.popWindows.get( otherAgentCommitment.ordinal() )).write( counterCurrentStateWithFixedPopA + "\t" + "RA" + "\n" );
				if (!state.frozenScenario){
					// censored discoveries
					if (counterForDiscovery > 0){
						state.bw2.write(counterForDiscovery + "\t" + "CDA" + "\n");
						state.bw2.write(counterForDiscovery + "\t" + "CDB" + "\n");
					}
				}
				return;
			}
			if (transition.equals("RB")){
				state.bwsMap.get("RB").get(state.popWindows.get( otherAgentCommitment.ordinal() )).write( counterCurrentStateWithFixedPopB + "\t" + "RB" + "\n" );
				if (!state.frozenScenario){
					// censored discoveries
					if (counterForDiscovery > 0){
						state.bw2.write(counterForDiscovery + "\t" + "CDA" + "\n");
						state.bw2.write(counterForDiscovery + "\t" + "CDB" + "\n");
					}
				}
				return;
			}
			if (transition.equals("CRA")){
				state.bwsMap.get("RA").get(state.popWindows.get(CommitmentState.OPTION_1.ordinal())).write( counterCurrentStateWithFixedPopA + "\t" + "CRA" + "\n" );
				return;
			}
			if (transition.equals("CRB")){
				state.bwsMap.get("RB").get(state.popWindows.get(CommitmentState.OPTION_2.ordinal())).write( counterCurrentStateWithFixedPopB + "\t" + "CRB" + "\n" );
				return;
			}
			if (transition.equals("IA")){
				state.bwsMap.get("IA").get(state.popWindows.get(CommitmentState.OPTION_1.ordinal())).write( counterCurrentStateWithFixedPopA + "\t" + "IA" + "\n" );
				return;
			}
			if (transition.equals("IB")){
				state.bwsMap.get("IB").get(state.popWindows.get(CommitmentState.OPTION_2.ordinal())).write( counterCurrentStateWithFixedPopB + "\t" + "IB" + "\n" );
				return;
			}
			if (transition.equals("CIA")){
				state.bwsMap.get("IA").get(state.popWindows.get(CommitmentState.OPTION_1.ordinal())).write( counterCurrentStateWithFixedPopA + "\t" + "CIA" + "\n" );
				return;
			}
			if (transition.equals("CIB")){
				state.bwsMap.get("IB").get(state.popWindows.get(CommitmentState.OPTION_2.ordinal())).write( counterCurrentStateWithFixedPopB + "\t" + "CIB" + "\n" );
				return;
			}
			if (transition.equals("CQ")){
				state.bw3.write(counterActive + "\t" + "CQ" + "\t" + commState + "\n");
			}
			if (transition.equals("CE")){
				state.bw3.write(counterInactive + "\t" + "CE" + "\t" + commState + "\n");
			}
			
		} catch (IOException e) {
			System.err.println("Failed to write the transition " + transition + "to file " + state.transitionsFilename);
			e.printStackTrace();
		}
	}
	
	private void resetAllCounters() {
		counterCurrentState = 0;
		counterForDiscovery = 0;
		counterForAlpha = 0;
		counterCurrentStateWithFixedPopA = 0;
		counterCurrentStateWithFixedPopB = 0;
	}
	

	private void estimateOptionQuality(BestFood state, CommitmentState option) {
		switch (option){
		case OPTION_1: {
			estimatedQuality = state.qualityArea1;
			estimatedQuality += state.errorVariance * state.random.nextGaussian();
			break;
		}
		case OPTION_2: {
			estimatedQuality = state.qualityArea2;
			estimatedQuality += state.errorVariance * state.random.nextGaussian();
			break;
		}
		default: {
			break;
		}
		}
	}
	
	/** Calculate the new position and move the agent */
	private MutableDouble2D moveAgent(BestFood state, MutableDouble2D movement) {
		// if movement is ZERO length I don't perform any operation
		if (movement.length() == 0){ return movement; }
		
		// (i) Resize the movement to the step-size = time * speed 
		movement.resize(state.speed * state.timeStepLength);
		
		// (ii) add the movement to my Position 
		MutableDouble2D newPosition = new MutableDouble2D().add(myPosition, movement);
		
		boolean collisionDetected = false;
		// (iii) check if inside the arena
		if ( !insideCircle(new Double2D(newPosition), state.arenaPosition, state.arenaSize) ){
			// If outside the arena,
			// a) I replace the agent position to the closest point on the arena edge
			MutableDouble2D vectorFromCenterArenaToMe = new MutableDouble2D().subtract(newPosition, state.arenaPosition);
			vectorFromCenterArenaToMe.resize(state.arenaSize/2.0);
			newPosition = vectorFromCenterArenaToMe.addIn(state.arenaPosition);
			// b) update the movement vector (needed later) wrt the new position
			movement = new MutableDouble2D().subtract(newPosition, myPosition);
			// c) trigger the collision sensor
			collisionDetected = true;
		}
		
		// (iv) if the agent is ACTIVE, he cannot exit the NEST
		if ( (activeState == ActiveState.ACTIVE || activeState == ActiveState.PREACTIVE) && 
				!insideCircle(new Double2D(newPosition), state.arenaPosition, state.nestSize) ){
			// If outside the nest:
			// a) I replace the agent position to the closest point on the NEST edge
			MutableDouble2D vectorFromCenterArenaToMe = new MutableDouble2D().subtract(newPosition, state.arenaPosition);
			vectorFromCenterArenaToMe.resize(state.nestSize/2.0 * 0.99);
			newPosition = vectorFromCenterArenaToMe.addIn(state.arenaPosition);
			// b) update the movement vector (needed later) wrt the new position
			movement = new MutableDouble2D().subtract(newPosition, myPosition);
			// c) trigger the collision sensor
			collisionDetected = true;
		}
		
		// (v) update the orientation
		previousOrientation = orientation;
		orientation = (collisionDetected)? -getAngle(movement) : getAngle(movement);
		
		// (vi) move the agent: that means update the new position of the agent
		state.arena.setObjectLocation(this, new Double2D(newPosition) );
		
		return movement;
	}
	
	
	// Communication to neighbors of the vector to area, my position and my orientation
	private void sendMessageToNeighbors(BestFood state) {
		Message myMex = new Message(ID, timeStep, myPosition, orientation, 
				new Double2D(vectorFromMeToFood), commitmentState, functionForPRho(estimatedQuality, state), 
				functionForPSigma(estimatedQuality, state), estimatedQuality);
		for (Object neigh:neighborsBag){
			if (neigh != this ){ // if the neighbor is not me 
				((BFAgent)neigh).addToInbox(myMex);
			}
		}
		
	}
	
	private void readMessage(BestFood state) {
		/* reset values */
		otherAgentCommitment = null;
		otherAgentTransitionProbability = 0;
		
		Collections.shuffle(inboxMail, new Random(state.random.nextLong(1000000)) );
		
		/* if I have at least one message */
		if (!inboxMail.isEmpty()){
			/* I take the first message */
			Message mex = inboxMail.get(0);

			switch (commitmentState) {
			case UNCOMMITTED: {
				otherAgentCommitment = mex.commitmentState;
				otherAgentTransitionProbability = mex.prho;
				otherAgentEstimatedQuality = mex.estimatedQuality;
				otherAgentVectorToFood = new MutableDouble2D(mex.vectorFromMeToFood);
				otherAgentPosition = mex.agentPosition;
				otherAgentOrientation = mex.agentOrientation;
				break;
			}
			case OPTION_1: {
				otherAgentCommitment = mex.commitmentState;
				otherAgentTransitionProbability = mex.psigma;
				otherAgentEstimatedQuality = mex.estimatedQuality;
				break;
			}
			case OPTION_2: {
				otherAgentCommitment = mex.commitmentState;
				otherAgentTransitionProbability = mex.psigma;
				otherAgentEstimatedQuality = mex.estimatedQuality;
				break;
			}
			}
		}
	}
	
	// Clear the inboxMail
	private void clearInboxMail() {
		inboxMail.clear();
		inboxMail.addAll(inboxMail_future);
		inboxMail_future.clear();
	}
	
	private MutableDouble2D translateHisVectorInMyRefFrame(MutableDouble2D hisVectorToTarget, double hisOrientation, Double2D vectorFromMeToHim){
		// rotate hisVector to real world reference frame
		hisVectorToTarget.rotate(hisOrientation);
		
		// calculate the vector from Me to the Target: MeToTarget = MeToHim + HimToTarget
		MutableDouble2D resultVector = new MutableDouble2D().add(vectorFromMeToHim, hisVectorToTarget);
		
		// translate the vector in my reference system: rotate the vector in opposite of my absolute orientation 
		resultVector.rotate(-orientation);
		
		return resultVector;
	}
	
	/*
	 * Uncorrelated Random Walk
	 */
	private MutableDouble2D uncorrelatedRandomWalk(BestFood state ) {
		MutableDouble2D randomWalkVector = new MutableDouble2D( 1.0, 0.0 );
		// random component
		randomWalkVector.rotate( state.random.nextDouble() *  Math.PI * 2.0 );
		// resize to unit size
		randomWalkVector.resize(1.0);
		
		return randomWalkVector;
	}
	
	/** Update of the target vectors upon agent's movement */
	private void updateVectorsToTargets(MutableDouble2D movement) {
		// rotate the movement vector in the previous coordinate system (local system with respect to the agent) 
		movement.rotate(-previousOrientation);
		// get the change performed in orientation
		double orientationChange = orientation - previousOrientation;
		// translation of the vector
		vectorFromMeToNest.subtractIn(movement);
		vectorFromMeToFood.subtractIn(movement);
		// rotation of the vector with respect to the new orientation
		vectorFromMeToNest.rotate(-orientationChange);
		vectorFromMeToFood.rotate(-orientationChange);
	}
	
	/*
	 * Calculation of the perfect vector that connects the agent to the center of the target area
	 */
	private void updateVectorsToTargets_Perfect(BestFood state) {
		switch (groundSensorReading){
		case OVER_NOTHING: {
			return;
		}
		case OVER_NEST: {
			Double2D vectorToArea = state.circularArena.getObjectLocation(state.nest);
			vectorFromMeToNest = new MutableDouble2D(vectorToArea.subtract(myPosition));
			vectorFromMeToNest.rotate(-orientation);
			return;
		}
		case OVER_FOOD1: {
			if (commitmentState == CommitmentState.OPTION_2) return; 
			Double2D vectorToArea = state.circularArena.getObjectLocation(state.area1);
			vectorFromMeToFood = new MutableDouble2D(vectorToArea.subtract(myPosition));
			vectorFromMeToFood.rotate(-orientation);
			return;
		}
		case OVER_FOOD2: {
			if (commitmentState == CommitmentState.OPTION_1) return; 
			Double2D vectorToArea = state.circularArena.getObjectLocation(state.area2);
			vectorFromMeToFood = new MutableDouble2D(vectorToArea.subtract(myPosition));
			vectorFromMeToFood.rotate(-orientation);
			return;
		}
		}
	}
	
	private void addToInbox(Message receivedMex) {
		if (receivedMex.timeStep == timeStep)
			inboxMail.add(receivedMex);
		else
			inboxMail_future.add(receivedMex);			
	}
	
	/*
	 * This method check if the agent is inside the one of the 'hot' areas 
	 */
	private void readGroundSensor(BestFood state) {
		groundSensorReading = GroundSensorReading.OVER_NOTHING;
		// inside Nest
		if (insideCircle(state.arena.getObjectLocation(this), state.circularArena.getObjectLocation(state.nest), state.nestSize ) ){
			groundSensorReading = GroundSensorReading.OVER_NEST;
			return;
		}
		// inside option1
		if (insideCircle(state.arena.getObjectLocation(this), state.circularArena.getObjectLocation(state.area1), state.area1size ) ){
			groundSensorReading = GroundSensorReading.OVER_FOOD1;
			return;
		}
		// inside option2
		if (insideCircle(state.arena.getObjectLocation(this), state.circularArena.getObjectLocation(state.area2), state.area2size ) ){
			groundSensorReading = GroundSensorReading.OVER_FOOD2;
			return;
		}
	}
	
	public boolean insideCircle(Double2D position, Double2D objectPosition, double objectDiameter){
		return ( (objectDiameter * objectDiameter / 4.0) -
				 ((position.x-objectPosition.x)*(position.x-objectPosition.x) + 
				  (position.y-objectPosition.y)*(position.y-objectPosition.y) ) > -BestFood.MIN_DIFFERENCE_THRESHOLD ) ;
	}
	
	public static Double2D Subtraction(MutableDouble2D vec1, MutableDouble2D vec2){
		return new Double2D(vec1.x - vec2.x, vec1.y - vec2.y);
	}
	
	public static Double2D Subtraction(Double2D vec1, Double2D vec2){
		return new Double2D(vec1.x - vec2.x, vec1.y - vec2.y);
	}

	
	/*****************************************************************************/
	/************************** FORZEN POPULATION CODE ***************************/
	/*****************************************************************************/
	
	public void assignAgentToPopulation(BestFood state) {
		if (ID < state.fp_numAgentsStartA){
			/* assign agent to option */
			assignToOption(state, CommitmentState.OPTION_1);
		}
		if (ID >= state.fp_numAgentsStartA && ID < (state.fp_numAgentsStartA + state.fp_numAgentsStartB)){
			/* assign agent to option */
			assignToOption(state, CommitmentState.OPTION_2);			
		}
	}
	
	public void assignToOption(BestFood state, CommitmentState opt){
		/** set commitment state*/
		commitmentState = opt;
		/** give area position knowledge */
		Double2D vectorToArea = new Double2D();
		if (opt == CommitmentState.OPTION_1){
			estimatedQuality = state.qualityArea1;
			vectorToArea = state.circularArena.getObjectLocation(state.area1);
		}
		if (opt == CommitmentState.OPTION_2){
			estimatedQuality = state.qualityArea2;
			vectorToArea = state.circularArena.getObjectLocation(state.area2);
		}
		vectorFromMeToFood = new MutableDouble2D(vectorToArea.subtract( state.arena.getObjectLocation(this) ));
		vectorFromMeToFood.rotate(-orientation);
		/** assign a random next destination */
		nextDestination = (state.random.nextDouble() < 0.5)? BehaviouralState.GO_NEST : BehaviouralState.GO_FOOD;
	}
	
	/*****************************************************************************/
	/*****************************************************************************/
}
