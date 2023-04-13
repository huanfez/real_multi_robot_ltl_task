package taskPlanning1;
/*
 * This task allocation automaton package achieves the following works:
 * 1. obtain the decompositions of a global task automaton;
 * 2. associate robots into transitions of each decomposition, which 
 *    synthesizes the subtask allocation automaton;
 * 3. find the optimal path inside subtask allocation automaton
 * 4. map paths into each robot so that each has the complete task 
 *    sequence to execute.
 * Note: robot was not considered a transition system in this package.*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import dk.brics.automaton.Automaton;
import dk.brics.automaton.BasicAutomata;
import dk.brics.automaton.BasicOperations;
import dk.brics.automaton.RegExp;
import dk.brics.automaton.ShuffleOperations;
import dk.brics.automaton.SpecialOperations;
import dk.brics.automaton.decompAut;
import dk.brics.automaton.State;
import dk.brics.automaton.StatePair;
import dk.brics.automaton.Transition;

public class TaskAllocAut {
	
	/* Get shortest paths of sub-automaton */
	public static String[] getShortestPath(List<Automaton> autSets) {
		String[] shortestPath = new String[autSets.size()];
		int i = 0;
		
		for (Automaton subAut : autSets)
			shortestPath[i++]= BasicOperations.getShortestExample(subAut, true);
		
		return shortestPath;
	}
	
	/* Generate action set with robots:
	 * Transform the composite event (robotID, event)
	 * into a single letter in the Alphabet */
	public static List<Action> getActionSet(List<char[][]> robotSet){
		List<Action> actionSet = new ArrayList<Action>();
		int id = 0;
		Character automatonEvent = 'A';
		int j = 0;
		
		for (char[][] robotObj : robotSet) {
			for (char[] actions : robotObj) {
				Action tmpAct = new Action(id, actions[0], actions[1], (char)((int)automatonEvent.charValue() + j++));
				actionSet.add(tmpAct);
				//automatonEvent = automatonEvent.charValue();
			}
			id++;
		}
		
		return actionSet;
	}

	/* Find alphabet related action*****
	 * Given an action described by a single letter in Alphabet, *
	 * locate robot's ID and event, i.e., where this action comes from */
	public static Action actionInvMap(Transition autEvent, List<Action> actionSet){
		int index = autEvent.getMin() - 'A';
		
		return actionSet.get(index);
	}
	
	/* Get node index******/
	private static int getNodeIndex(State stat, Node[] nodeArray) {
		int nodeIndex = 0;
		
		for (nodeIndex = 0; nodeIndex < nodeArray.length; nodeIndex++)
			if (stat.toString().equals(nodeArray[nodeIndex].value))
				break;
		
		return nodeIndex;
	}
	
	/* Generate task automaton *****
	 * Parallel compose two paths (strings) to generate an automaton */
	public static Automaton makeTaskAut(String[] shortPaths, List<Action> actionSet) {
		Automaton taskAutomaton = new Automaton();
		boolean Initial = true;
		
		for (String shortestPath : shortPaths) {
			if (Initial) {
				taskAutomaton = BasicAutomata.makeString(shortestPath);
				Initial = false;
			}
			else
				taskAutomaton = ShuffleOperations.shuffle(taskAutomaton, BasicAutomata.makeString(shortestPath));
		}
		
		return taskAutomaton;
	}
	
	/* Generate task allocation automaton with robots: *
	 * Given a task automaton and a set of robots with capabilities, 
	 * replace task automaton events with new transition events (<robotID, event> 
	 * described by a single letter). Consequently, a task allocation automaton is 
	 * generated *****/
	public static Automaton makeTaskAlloAut(Automaton taskAutomaton, List<Action> actionSet) {
		Automaton taskAllocAut = new Automaton();
		boolean Substitution = false;
		
		Set<Character> taskEventSet = decompAut.getEventSet(taskAutomaton);
		for (Character taskEvent : taskEventSet) {
			Map<Character, Set<Character>> actionMap = new HashMap<Character, Set<Character>>();
			Set<Character> mapCharacterSet = new HashSet<Character>();
			
			for (Action act : actionSet)
				if (act.event == taskEvent)
					mapCharacterSet.add(act.autEvent);
			
			actionMap.put(taskEvent, mapCharacterSet);
			//System.out.println(actionMap);
			if (Substitution == false) {
				taskAllocAut = SpecialOperations.subst(taskAutomaton, actionMap);
				Substitution = true;
			}
			else
				taskAllocAut = SpecialOperations.subst(taskAllocAut, actionMap);
		}
		System.out.println("The task allocation autoaton is :" + taskAllocAut);
		
		return taskAllocAut;
	}
	
	/* Search for optimal path*
	 * Use Dijkstra searching algorithm to return the optimal task allocation solution
	 * from the task allocation automaton (converted to a node-edge structure)*****/
	public static List<Character> dijkstraSearch(Automaton taskAllocAut, List<Action> actionSet){
		//initialize the graph		
		int NodeSize = taskAllocAut.getStates().size();
		Node autNode[] = new Node[NodeSize];	
		Node initNode = new Node(taskAllocAut.getInitialState().toString());
		Node endNode = new Node(taskAllocAut.getAcceptStates().toString());
		int i = 0;
		
		for (State allocStat : taskAllocAut.getStates()) {
			autNode[i] = new Node(allocStat.toString());
			autNode[i].adjacencies = new ArrayList<Edge>();
			i++;
		}
		
		i = 0;
		for (State allocStat : taskAllocAut.getStates()) {
			for (Transition tran : allocStat.getTransitions()) {
				int nextNodeIndex = getNodeIndex(tran.getDest(), autNode);
				Action edgeAction = actionInvMap(tran, actionSet);	
				autNode[i].adjacencies.add(new Edge(autNode[nextNodeIndex], edgeAction.weight, edgeAction.autEvent));
			}
			
			if (allocStat == taskAllocAut.getInitialState())
				initNode = autNode[i];
			
			if (taskAllocAut.getAcceptStates().contains(allocStat))
				endNode = autNode[i];
			
			i++;
		}

		//compute paths
		DijkstraAlgo.computePaths(initNode);

		//print shortest paths
		//List<Node> path = DijkstraAlgo.getShortestPathTo(endNode);
		List<Character> path = DijkstraAlgo.getShortestActPathTo(endNode);
		//System.out.println("Path: " + path);
		
		return path;
	}
	
	/* Map task allocation path into each robot: ****
	 * Map the task allocation solution into each robot */
	public static List<String> getTaskSpec(List<Character> taskPath, List<Action> taAutActs, int robotId) {
		List<String> taskSpecStrs = new ArrayList<String>();
		for (Character taskAlph: taskPath)
			if (taAutActs.get(taskAlph - 'A').robotID == robotId)
				taskSpecStrs.add("F(" + taAutActs.get(taskAlph - 'A').event + ")");
		
		return taskSpecStrs;
	}
	
	/* main program: Optimal task allocation path */
	public static void main(String[] args) throws IOException{
		
		//RegExp regExpr = new RegExp("a(b|d)");
		//RegExp regExpr = new RegExp("a(bd|db)|b(ad|da)|d(ab|ba)");
		//////////RegExp regExpr = new RegExp(args[0]);
		//RegExp regExpr = new RegExp("(bd)|(db)");
		
		RegExp regExpr = new RegExp("(a(ba)*c(dc)*)((d(cd)*b(ab)*)(a(ba)*c(dc)*))*");
		RegExp regExpr2 = new RegExp("(c(dc)*a(ba)*)((b(ab)*d(cd)*)(c(dc)*a(ba)*))*");
		Automaton unionAut = BasicOperations.union(regExpr.toAutomaton(), regExpr2.toAutomaton());
		System.out.println(regExpr2.toAutomaton());
		
		State InitState = unionAut.getInitialState();
		StatePair cnetState = new StatePair(InitState, InitState);
		Collection<StatePair> epsilPair = new ArrayList<StatePair>();
		epsilPair.add(cnetState);
		BasicOperations.addEpsilons(unionAut, epsilPair);
		Automaton globalAut = unionAut; //regExpr.toAutomaton();
		
		char robot1[][] = {{'a', 3}, {'b', 2}};
		char robot2[][] = {{'a', 3}, {'d', 2}};
		List<char[][]> robotSet = Arrays.asList(robot1, robot2);
		List<Action> taskAutActs = getActionSet(robotSet);
		
		List<Automaton> subAutSets = decompAut.paraDecomp(globalAut);
		//System.out.println(subAutSets);
		
		String[] ShortestPaths = getShortestPath(subAutSets);
		//System.out.println(ShortestPaths[1]);
		
		Automaton taskAllocAut = new Automaton();
		taskAllocAut = makeTaskAut(ShortestPaths, taskAutActs);
		//System.out.println(taskAllocAut);
		
		Automaton finalTaskAut = makeTaskAlloAut(taskAllocAut, taskAutActs);
		//System.out.println(finalTaskAut);

		//print shortest paths
		List<Character> path = dijkstraSearch(finalTaskAut, taskAutActs);
		System.out.println("The optimal task allocation Path: " + path);
		for (Character alpha : path)
			System.out.println("r" + (taskAutActs.get(alpha - 'A').robotID+1) + "," + taskAutActs.get(alpha - 'A').event);
		
		//output task specification for each robot
		List<String> taskSpecStrings;
		PrintWriter writer = new PrintWriter(new FileWriter("taskSpecification.txt"));
		
		for (int robotId = 0; robotId < 2; robotId++) {
			taskSpecStrings = getTaskSpec(path, taskAutActs, robotId);
			System.out.println(taskSpecStrings);
			writer.println("The specification of robot" + (robotId+1) + " is:");
			writer.println(taskSpecStrings);
		}
		
		writer.close();
			
	}

}

/* Define the robot action class */
class Action {
	char event;
	int robotID;
	double weight;
	Character autEvent;
	
	public Action(int robotID, char event, double weight, Character autEvent){
		this.robotID = robotID;
		this.event = event;
		this.weight = weight;
		this.autEvent = autEvent;
	}
	
	public Action() {};
}
