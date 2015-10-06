package nospatial;

import java.io.IOException;
import java.util.List;

import nospatial.NoSpatial.Transition;
import nospatial.NoSpatial.RandomDistributions;

public class NSAgent implements Cloneable{
	
	enum ControlState {
		UNCOMMITTED,
		OPTION_A,
		OPTION_B
	}
	
	enum ActionType {
		SPONTANEOUS,
		INTERACTION
	}
	
	enum SpatialState {
		ACTIVE,
		HOLIDAY
	};
	
	int ID;
	ControlState currentState;
	SpatialState activeIdleState;
	long timeOfEntranceInCurrentState;
	long timeOfEntranceInCurrentStateWithFixedPopulationA;
	long timeOfEntranceInCurrentStateWithFixedPopulationB;
	long timeOfTheLastInteraction;
	
	double PgammaA;
	double PgammaB;
	double PalphaA;
	double PalphaB;
	double PrhoA;
	double PrhoB;
	double PsigmaA;
	double PsigmaB;
	
	public NSAgent() {
		this(0);
	}
	
	public NSAgent(int id) {
		currentState = ControlState.UNCOMMITTED;
		activeIdleState = SpatialState.ACTIVE;
		timeOfEntranceInCurrentState = 0;
		timeOfEntranceInCurrentStateWithFixedPopulationA = 0;
		timeOfEntranceInCurrentStateWithFixedPopulationB = 0;
		timeOfTheLastInteraction = 0;
		ID = id;
		PgammaA = 0;
		PgammaB = 0;
		PalphaA = 0;
		PalphaB = 0;
		PrhoA = 0;
		PrhoB = 0;
		PsigmaA = 0;
		PsigmaB = 0;
	}
	
	@Override
	public NSAgent clone() {
		NSAgent clone;
	    try {
	    	clone = (NSAgent) super.clone();
	    }
	    catch (CloneNotSupportedException e) {
	        throw new Error();
	    }
	    clone.currentState = currentState;
		clone.activeIdleState = activeIdleState;
		clone.timeOfEntranceInCurrentState = timeOfEntranceInCurrentState;
		clone.timeOfEntranceInCurrentStateWithFixedPopulationA = timeOfEntranceInCurrentStateWithFixedPopulationA;
		clone.timeOfEntranceInCurrentStateWithFixedPopulationB = timeOfEntranceInCurrentStateWithFixedPopulationB;
		clone.timeOfTheLastInteraction = timeOfTheLastInteraction;
		clone.ID = ID;
		clone.PgammaA = PgammaA;
		clone.PgammaB = PgammaB;
		clone.PalphaA = PalphaA;
		clone.PalphaB = PalphaB;
	    clone.PrhoA = PrhoA;
	    clone.PrhoB = PrhoB;
	    clone.PsigmaA = PsigmaA;
	    clone.PsigmaB = PsigmaB;

	    return clone;
	}
	
	public void updateActiveIdleState(NoSpatial state){
		switch (currentState) {
		case OPTION_A:{
			if (state.random.nextDouble() < state.enterIdleStateProbA ){
				activeIdleState = SpatialState.HOLIDAY;
			} else {
				activeIdleState = SpatialState.ACTIVE;			
			}			
			break;
		}
		case OPTION_B:{
			if (state.random.nextDouble() < state.enterIdleStateProbB ){
				activeIdleState = SpatialState.HOLIDAY;
			} else {
				activeIdleState = SpatialState.ACTIVE;			
			}			
			break;
		}
		case UNCOMMITTED:{
			activeIdleState = SpatialState.ACTIVE;
			break;
		}
		}
	}
	
	public void makeTransitionsSequential(NoSpatial state, List<ActionType> thingsToDo, List<NSAgent> prevStepPopulation){
		for (ActionType action:thingsToDo){
			if (action == ActionType.SPONTANEOUS){
				if (makeIndividualTransition(state))
					break;
			}
			if (action == ActionType.INTERACTION){
				if (state.writeTrasitionTimesToFile){
					/** store effective interaction */
					try {
						state.bw3.write(NoSpatial.GetLetterOfMacrostate(currentState) + "\t" + "I" + "\t" + (state.timeStep - timeOfTheLastInteraction) + "\n");
						state.bw3.flush();
						timeOfTheLastInteraction = state.timeStep; 
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
				/** execute the interaction */
				if (makeIteractionTransition(state, prevStepPopulation))
					break;
			}
		}
	}
	
	public void makeTransitionsAllTogether(NoSpatial state, List<ActionType> thingsToDo, List<NSAgent> prevStepPopulation){
		if (thingsToDo.contains(ActionType.SPONTANEOUS) && !thingsToDo.contains(ActionType.INTERACTION)){
			makeIndividualTransition(state);
		}
		if (!thingsToDo.contains(ActionType.SPONTANEOUS) && thingsToDo.contains(ActionType.INTERACTION)){
			makeIteractionTransition(state, prevStepPopulation);
		}
		if (thingsToDo.contains(ActionType.SPONTANEOUS) && thingsToDo.contains(ActionType.INTERACTION)){
			makeBothTypeTransitions(state, prevStepPopulation);
		}
	}
	
	@SuppressWarnings("unused")
	private void resampleStepFunction(NoSpatial state) {
		/* recompute each time the step function */
		double switchingPoint = state.computeSwitchingPoint();
		if (state.useOptQualityToComputeDiscovery){
			double pVMin = state.computePMinMax(state.randomDistribution, state.vMin, state.gammaA);
			double pVMax = state.computePMinMax(state.randomDistribution, state.vMax, state.gammaA);
			PgammaA = (state.vA > switchingPoint)? pVMax : pVMin;
			PgammaB = (state.vB > switchingPoint)? pVMax : pVMin;
		}
		if (state.useOptQualityToComputeAbandonment){
			double switchingPointAlpha = state.computeSwitchingPoint(RandomDistributions.INVERSE_FRACTION);
			double pVMin = state.computePMinMax(RandomDistributions.INVERSE_FRACTION, state.vMin, state.alphaA);
			double pVMax = state.computePMinMax(RandomDistributions.INVERSE_FRACTION, state.vMax, state.alphaA);
			PalphaA = (state.vA > switchingPointAlpha)? pVMax : pVMin;
			PalphaB = (state.vB > switchingPointAlpha)? pVMax : pVMin;
		}
		if (state.useOptQualityToComputeRecruitment){
			double pVMin = state.computePMinMax(state.randomDistribution, state.vMin, state.rhoA);
			double pVMax = state.computePMinMax(state.randomDistribution, state.vMax, state.rhoA);
			PrhoA = (state.vA > switchingPoint)? pVMax : pVMin;
			PrhoB = (state.vB > switchingPoint)? pVMax : pVMin;
		}
		if (state.useOptQualityToComputeCrossinhibition){
			double pVMin = state.computePMinMax(state.randomDistribution, state.vMin, state.sigmaA);
			double pVMax = state.computePMinMax(state.randomDistribution, state.vMax, state.sigmaA);
			PsigmaA = (state.vA > switchingPoint)? pVMax : pVMin;
			PsigmaB = (state.vB > switchingPoint)? pVMax : pVMin;
		}
		
	}

	public boolean makeBothTypeTransitions(NoSpatial state, List<NSAgent> prevStepPopulation) {
		/** Select another agent (different from me and ACTIVE) */
		NSAgent neigh = this;
		boolean foundAnActiveNeigh = true;
		int i = 0;
		while (neigh.ID == this.ID || (currentState != ControlState.UNCOMMITTED && neigh.activeIdleState == SpatialState.HOLIDAY)){
			/** Avoid to spend too much time sampling neighbors */ 
			i++;
			if (i > prevStepPopulation.size()*2){
				foundAnActiveNeigh = false;
				break;
			}
			/** Sample a random neighbor */
			int a = state.random.nextInt(prevStepPopulation.size());
			neigh = prevStepPopulation.get(a);
		}
		double randNum = state.random.nextDouble();
		long timeSpent = state.timeStep - timeOfEntranceInCurrentState;
		long timeSpentPopA = state.timeStep - timeOfEntranceInCurrentStateWithFixedPopulationA;
		long timeSpentPopB = state.timeStep - timeOfEntranceInCurrentStateWithFixedPopulationB;
		boolean changedState = false;
		switch (currentState){
		case UNCOMMITTED:{
			/* I change my PrhoA and PrhoB taking them from my neighbor */
			double pRhoA_Received = (foundAnActiveNeigh && neigh.currentState == ControlState.OPTION_A)? neigh.PrhoA : 0; 
			double pRhoB_Received = (foundAnActiveNeigh && neigh.currentState == ControlState.OPTION_B)? neigh.PrhoB : 0;
			
			/* DISCOVERY A */
			if (randNum < PgammaA) {
				currentState = ControlState.OPTION_A;
				storeTransition(state, "DA", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			/* DISCOVERY B */
			if (randNum >= PgammaA && randNum < (PgammaA + PgammaB)) {
				currentState = ControlState.OPTION_B;
				storeTransition(state, "DB", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			/* GET RECRUITED BY A */
			if (randNum >= (PgammaA + PgammaB) && randNum < (PgammaA + PgammaB + pRhoA_Received) ){
				currentState = ControlState.OPTION_A;					
				storeTransition(state, "RA", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
				
				if (state.writeTempFile){
					try {
						state.bwTmp.write(neigh.ID + "\t" + ID + "\t" + "A" + "\t" + "R" + "\n");
					} catch (IOException e) { e.printStackTrace(); }
				}
			}
			/* GET RECRUITED BY B */
			if (randNum >= (PgammaA + PgammaB + pRhoA_Received) && randNum < (PgammaA + PgammaB + pRhoA_Received + pRhoB_Received) ){
				currentState = ControlState.OPTION_B;
				storeTransition(state, "RB", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
				
				if (state.writeTempFile){
					try {
						state.bwTmp.write(neigh.ID + "\t" + ID + "\t" + "B" + "\t" + "R" + "\n");
					} catch (IOException e) { e.printStackTrace(); }
				}
			}
			break;
		}
		case OPTION_A:{
			/* I change the pSigmaB taking it from my neighbor (if he is OPTION_B) */
			double pSigmaB_Received = (neigh.currentState == ControlState.OPTION_B)? neigh.PsigmaB : 0; 
			
			/* DECAY */
			if (randNum < PalphaA){
				currentState = ControlState.UNCOMMITTED;					
				storeTransition(state, "LA", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			/* CROSS-INHIBITED BY B */
			if (randNum >= PalphaA && randNum < (PalphaA + pSigmaB_Received)){
				if (foundAnActiveNeigh && neigh.currentState == ControlState.OPTION_B){
					currentState = ControlState.UNCOMMITTED;					
					storeTransition(state, "IB", timeSpent, timeSpentPopA, timeSpentPopB);
					changedState = true;
					
					if (state.writeTempFile){
						try {
							state.bwTmp.write(neigh.ID + "\t" + ID + "\t" + "B" + "\t" + "I" + "\n");
						} catch (IOException e) { e.printStackTrace(); }
					}
				}
			}
			break;
		}
		case OPTION_B:{
			/* I change the pSigmaB taking it from my neighbor (if he is OPTION_B) */
			double pSigmaA_Received = (neigh.currentState == ControlState.OPTION_A)? neigh.PsigmaA : 0;
			
			/* DECAY */
			if (randNum < PalphaB){
				currentState = ControlState.UNCOMMITTED;
				storeTransition(state, "LB", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			/* CROSS-INHIBITED BY A */
			if (randNum >= PalphaB && randNum < (PalphaB + pSigmaA_Received)){
				if (foundAnActiveNeigh && neigh.currentState == ControlState.OPTION_A){
					currentState = ControlState.UNCOMMITTED;					
					storeTransition(state, "IA", timeSpent, timeSpentPopA, timeSpentPopB);
					changedState = true;
					if (state.writeTempFile){
						try {
							state.bwTmp.write(neigh.ID + "\t" + ID + "\t" + "A" + "\t" + "I" + "\n");
						} catch (IOException e) { e.printStackTrace(); }
					}
				}
			}
			break;
		}
		}
		return changedState;
	}
	
	public boolean makeIndividualTransition(NoSpatial state) {
		double randNum = state.random.nextDouble();
		long timeSpent = state.timeStep - timeOfEntranceInCurrentState;
		long timeSpentPopA = state.timeStep - timeOfEntranceInCurrentStateWithFixedPopulationA;
		long timeSpentPopB = state.timeStep - timeOfEntranceInCurrentStateWithFixedPopulationB;
		boolean changedState = false;
		switch (currentState){
		case UNCOMMITTED:{
			/* DISCOVERY A */
			if (randNum < PgammaA) {
				currentState = ControlState.OPTION_A;
				storeTransition(state, "DA", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			/* DISCOVERY B */
			if (randNum >= PgammaA && randNum < (PgammaA + PgammaB)) {
				currentState = ControlState.OPTION_B;
				storeTransition(state, "DB", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			break;
		}
		case OPTION_A:{
			/* DECAY */
			if (randNum < state.alphaA){
				currentState = ControlState.UNCOMMITTED;					
				storeTransition(state, "LA", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			break;
		}
		case OPTION_B:{
			/* DECAY */
			if (randNum < state.alphaB){
				currentState = ControlState.UNCOMMITTED;					
				storeTransition(state, "LB", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			break;
		}
		}
		return changedState;
	}

	public boolean makeIteractionTransition(NoSpatial state, List<NSAgent> prevStepPopulation) {
		/* select another agent (different from me) */
		NSAgent neigh = this;
		while (neigh.ID == this.ID){
			int a = state.random.nextInt(prevStepPopulation.size());
			neigh = prevStepPopulation.get(a);
		}
		
		long timeSpent = state.timeStep - timeOfEntranceInCurrentState;
		long timeSpentPopA = state.timeStep - timeOfEntranceInCurrentStateWithFixedPopulationA;
		long timeSpentPopB = state.timeStep - timeOfEntranceInCurrentStateWithFixedPopulationB;
		boolean changedState = false;
		switch (currentState){
		case UNCOMMITTED:{
			double randNum = state.random.nextDouble();
			if (neigh.currentState == ControlState.OPTION_A && randNum < PrhoA){
				/* GET RECRUITED BY A */
				currentState = ControlState.OPTION_A;					
				storeTransition(state, "RA", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			if (neigh.currentState == ControlState.OPTION_B && randNum < PrhoB){
				/* GET RECRUITED BY B */
				currentState = ControlState.OPTION_B;
				storeTransition(state, "RB", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			break;
		}
		case OPTION_A:{
			double randNum = state.random.nextDouble();
			if (neigh.currentState == ControlState.OPTION_B && randNum < state.sigmaB){
				/* CROSS-INHIBITED BY B */
				currentState = ControlState.UNCOMMITTED;					
				storeTransition(state, "IB", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			break;
		}
		case OPTION_B:{
			double randNum = state.random.nextDouble();
			if (neigh.currentState == ControlState.OPTION_A && randNum < state.sigmaA){
				/* CROSS-INHIBITED BY A */
				currentState = ControlState.UNCOMMITTED;					
				storeTransition(state, "IA", timeSpent, timeSpentPopA, timeSpentPopB);
				changedState = true;
			}
			break;
		}
		}
		return changedState;
	}
	
	private void storeTransition(NoSpatial state, String trans, long timeSpent, long timeSpentPopA, long timeSpentPopB) {
		if (trans.equals("DA")){
			/* store transition */
			state.linearTransitions.add(new Transition("U","DA",timeSpent));
			/* store censored data */
			state.nonLinearTransitions.add(new Transition("U","CRA",timeSpentPopA));
			state.nonLinearTransitions.add(new Transition("U","CRB",timeSpentPopB));
			/* update population sizes */
			state.popU--;
			state.popA++;
			if (state.writeTrasitionTimesToFile){
				/* store the censored interaction */
				try {
					state.bw3.write("U" + "\t" + "C" + "\t" + (state.timeStep - timeOfTheLastInteraction) + "\n");
					state.bw3.flush();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		if (trans.equals("DB")){
			/* store transition */
			state.linearTransitions.add(new Transition("U","DB",timeSpent));
			/* store censored data */
			state.nonLinearTransitions.add(new Transition("U","CRA",timeSpentPopA));
			state.nonLinearTransitions.add(new Transition("U","CRB",timeSpentPopB));
			/* update population sizes */
			state.popU--;
			state.popB++;
			if (state.writeTrasitionTimesToFile){
				/* store the censored interaction */
				try {
					state.bw3.write("U" + "\t" + "C" + "\t" + (state.timeStep - timeOfTheLastInteraction) + "\n");
					state.bw3.flush();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		if (trans.equals("LA")){
			/* store transition */
			state.linearTransitions.add(new Transition("A","LA",timeSpent));
			/* store censored data */
			state.nonLinearTransitions.add(new Transition("A","CIB",timeSpentPopB));
			/* update population sizes */
			state.popA--;
			state.popU++;
			if (state.writeTrasitionTimesToFile){
				/* store the censored interaction */
				try {
					state.bw3.write("A" + "\t" + "C" + "\t" + (state.timeStep - timeOfTheLastInteraction) + "\n");
					state.bw3.flush();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		if (trans.equals("LB")){
			/* store transition */
			state.linearTransitions.add(new Transition("B","LB",timeSpent));
			/* store censored data */
			state.nonLinearTransitions.add(new Transition("B","CIA",timeSpentPopA));
			/* update population sizes */
			state.popB--;
			state.popU++;
			if (state.writeTrasitionTimesToFile){
				/* store the censored interaction */
				try {
					state.bw3.write("B" + "\t" + "C" + "\t" + (state.timeStep - timeOfTheLastInteraction) + "\n");
					state.bw3.flush();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		if (trans.equals("RA")){
			/* store transition */
			state.nonLinearTransitions.add(new Transition("U","RA",timeSpentPopA));
			/* store the censored data for discovery and other recruitment */
			state.linearTransitions.add(new Transition("U","CD",timeSpent));
			state.nonLinearTransitions.add(new Transition("U","CRB",timeSpentPopB));
			/* update population sizes */
			state.popU--;
			state.popA++;
		}
		if (trans.equals("RB")){
			/* store transition */
			state.nonLinearTransitions.add(new Transition("U","RB",timeSpentPopB));
			/* store the censored data for discovery and other recruitment */
			state.linearTransitions.add(new Transition("U","CD",timeSpent));
			state.nonLinearTransitions.add(new Transition("U","CRA",timeSpentPopA));
			/* update population sizes */
			state.popU--;
			state.popB++;
		}
		if (trans.equals("IA")){
			/* store transition */
			state.nonLinearTransitions.add(new Transition("B","IA",timeSpentPopA));
			/* store the censored data */
			state.linearTransitions.add(new Transition("B","CLB",timeSpent));
			/* update population sizes */
			state.popB--;
			state.popU++;
		}
		if (trans.equals("IB")){
			/* store transition */
			state.nonLinearTransitions.add(new Transition("A","IB",timeSpentPopB));
			/* store the censored data */
			state.linearTransitions.add(new Transition("A","CLA",timeSpent));
			/* update population sizes */
			state.popA--;
			state.popU++;
		}
		/* reset counters */
		timeOfEntranceInCurrentState = state.timeStep;
		timeOfEntranceInCurrentStateWithFixedPopulationA = state.timeStep; 
		timeOfEntranceInCurrentStateWithFixedPopulationB = state.timeStep;
		timeOfTheLastInteraction = state.timeStep;
	}

}
