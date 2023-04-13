package weightedModelChecking;

import org.jgrapht.*;
import org.jgrapht.alg.util.NeighborCache;
import org.jgrapht.graph.*;

import java.util.*;

import dk.brics.automaton.Automaton;
import dk.brics.automaton.RegExp;
import dk.brics.automaton.State;
import dk.brics.automaton.Transition;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class TransitionSystem {
	/*************transition systems********/
	/**Transition System1 **/		
	public static RobotAction[] robotActionSet1 = {new RobotAction(1, "e", 0.3), 
			new RobotAction(1, "a", 1.0), new RobotAction(1, "b", 1.0)};
	
	public static Graph<String, DefaultEdge> transitionSystem1 = new DefaultDirectedGraph<String, 
			DefaultEdge>(DefaultEdge.class);
	static
	{
		for (RobotAction robotAction : robotActionSet1)
			transitionSystem1.addVertex(robotAction.event);
	    
		transitionSystem1.addEdge("e", "e");
		transitionSystem1.addEdge("e", "a");
		transitionSystem1.addEdge("e", "b");
		transitionSystem1.addEdge("a", "b");
		transitionSystem1.addEdge("b", "e");
	};
	/****************************************/
	
	/**Find cost of robot's certain action**/
	public static double findWeightOfRobotAction(String action, RobotAction[] robotActionSet) {
		for (RobotAction robotAction : robotActionSet)
			if (robotAction.event.equals(action) )
					return robotAction.weight;
		return 0;
	}
	
	/**Extract first occurrence of integer from string*****/
	public static String extractInt(String state) {
		Pattern p = Pattern.compile("([\\d]+)");
		Matcher m = p.matcher(state);
		if (m.find()) {
			//System.out.println(m.group(0));
			return m.group(0);
		}
		
		return null;
	}
	
	/**Product automaton of transition system and task automaton**/
	public static DirectedWeightedPseudograph<ArrayList<String>,String> TsAutomatonProduct(
			Graph<String, DefaultEdge> transitionSystem, Automaton taskAutomaton, RobotAction[] robotActSet){
		DirectedWeightedPseudograph<ArrayList<String>,String> productAutomaton = 
				new DirectedWeightedPseudograph<ArrayList<String>,String>(String.class);
		DirectedPseudograph<String,String> taskAutomatonGraph = 
				new DirectedPseudograph<String,String>(String.class);
		
		for (State state : taskAutomaton.getStates()) {
			taskAutomatonGraph.addVertex(extractInt(state.toString()));
			for (Transition transition : state.getTransitions()) {
				taskAutomatonGraph.addVertex(extractInt(transition.getDest().toString()));
				for (int event = (int)transition.getMin(); event <= (int)transition.getMax(); event++) {
					taskAutomatonGraph.addEdge(extractInt(state.toString()), 
							extractInt(transition.getDest().toString()), Character.toString((char)event));
							
					for (String stateOfTs : transitionSystem.vertexSet())
						if (Character.toString((char)event).equals(stateOfTs)) {
							ArrayList<String> vertexOfProdAut = new ArrayList<String>();
							vertexOfProdAut.add(stateOfTs);
							vertexOfProdAut.add(extractInt(transition.getDest().toString()));
							productAutomaton.addVertex(vertexOfProdAut);
						}
				}
			}
		}

		NeighborCache<String,String> vertexSuccessorSet = 
				new NeighborCache<String,String>(taskAutomatonGraph);
		NeighborCache<String,DefaultEdge> vertexTsSuccessorSet = 
				new NeighborCache<String,DefaultEdge>(transitionSystem);
		
		for (ArrayList<String> vertexOfProdAut : productAutomaton.vertexSet())
			for (String vertexSuccessor : vertexSuccessorSet.successorsOf(vertexOfProdAut.get(1)))
				for (String stateOfTs : vertexTsSuccessorSet.successorsOf(vertexOfProdAut.get(0)))
					if (taskAutomatonGraph.getAllEdges(vertexOfProdAut.get(1), 
							vertexSuccessor).contains(stateOfTs)) {
						ArrayList<String> targetVertex = new ArrayList<String>();
						targetVertex.add(stateOfTs);
						targetVertex.add(vertexSuccessor);
						productAutomaton.addEdge(vertexOfProdAut, targetVertex, stateOfTs);
						productAutomaton.setEdgeWeight(stateOfTs, findWeightOfRobotAction(stateOfTs,
								robotActSet));
					}
		
		/****Need to specify initial state and remove unreachable states******/
		/**********taskAutomaton.getAcceptStates();**************/
		
		return productAutomaton;
	}
	
	public static void main(String args[]) 
	{ 
		
		System.out.println(transitionSystem1);
		RegExp regExpr = new RegExp("e(b|a)");
		Automaton taskAutomaton = regExpr.toAutomaton();
		System.out.println(TsAutomatonProduct(transitionSystem1, taskAutomaton, robotActionSet1));
		
		Pattern p = Pattern.compile("([\\d]+)");
		Matcher m = p.matcher(taskAutomaton.getInitialState().toString());
		if (m.find()) {
			System.out.println(m.group(0));
		}
	}
}

/* Define the robot action class */
class RobotAction {
	String event;
	int robotID;
	double weight;
	
	public RobotAction(int robotID, String event, double weight){
		this.robotID = robotID;
		this.event = event;
		this.weight = weight;
	}
	
	public RobotAction() {};
}
