package spatial;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.util.Bag;
import sim.util.Double2D;

public class PreControlStepAgent implements Steppable {

	private static final long serialVersionUID = -2029648162384378726L;

	@Override
	public void step(SimState simState) {
		BestFood state = (BestFood) simState;
//		for(int x=0; x < state.arena.allObjects.numObjs; x++) {
//			ForagerAgent agent = (ForagerAgent)state.arena.allObjects.objs[x];
//			Double2D agentPosition = state.arena.getObjectLocation(agent);
//			Bag neighborsBag = new Bag();
//			/** Computing the neighbours' list only if the agent is inside the nest area */  
//			if ( agent.insideCircle(agentPosition, state.circularArena.getObjectLocation(state.nest), state.nestSize ) ){
//				neighborsBag = state.arena.getObjectsExactlyWithinDistance(agentPosition, state.localSensing, false, true, true, new Bag());
//			}
//			agent.setNeighborsBag(neighborsBag);
//		}
		Bag inNestAgentsBag = new Bag();
		Bag emptyBag = new Bag();
		for(int x=0; x < state.arena.allObjects.numObjs; x++) {
			BFAgent agent = (BFAgent)state.arena.allObjects.objs[x];
			Double2D agentPosition = state.arena.getObjectLocation(agent);
			/** Computing the neighbours' list only if the agent is inside the nest area */  
			if ( agent.insideCircle(agentPosition, state.circularArena.getObjectLocation(state.nest), state.nestSize ) ){
				inNestAgentsBag.add(agent);
			} else {
				agent.setNeighborsBag(emptyBag);
			}
		}
		for(int x=0; x < inNestAgentsBag.size(); x++) {
			((BFAgent)inNestAgentsBag.get(x)).setNeighborsBag(inNestAgentsBag);
		}
			
			
	}

}
