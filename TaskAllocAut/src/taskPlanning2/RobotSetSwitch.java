package taskPlanning2;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.alg.connectivity.BiconnectivityInspector;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import com.google.common.collect.Sets;

/** Robot configuration for a list of automata:
 * given a set of robots, the configuration guarantees a minimum switch 
 * of robot between these automata */
public class RobotSetSwitch {
	public List<RobotConfig> autRobotsPair = new ArrayList<>();
	
	/** Initialize with an automaton list and a set of robots, 
	 * automaton - robot set pairs are valued by an optimal configuration
	 * of robot sets */
	public RobotSetSwitch(List<TaskAutomaton> automatonList, Set<Character> CoopEventSet, 
			Set<Robot> providedRobots) {
		Set<List<Graph<RobotConfig, DefaultEdge>>> SetOfMaxIndepConnectedSubgraphs = maxGraphSetOfRobotConfig(
				automatonList, providedRobots, TaskAutomaton.charSet2StringList(CoopEventSet));
		this.autRobotsPair = optListOfRobotConfig(CoopEventSet, SetOfMaxIndepConnectedSubgraphs);
	}
	
	/** New initialization equation */
	public RobotSetSwitch(List<Set<RobotConfig>> ListOfMinRobotConfigSet, Set<Character> CoopEventSet) {
		Set<List<Graph<RobotConfig, DefaultEdge>>> SetOfMaxIndepConnectedSubgraphs = 
				maxGraphSetOfRobotConfig(ListOfMinRobotConfigSet);
		this.autRobotsPair = optListOfRobotConfig(CoopEventSet, SetOfMaxIndepConnectedSubgraphs);
	}
	
	public RobotSetSwitch(List<RobotConfig> autRobotsPair){
		this.autRobotsPair = autRobotsPair;
	}
			
	/** Optimal robot configuration for automaton list: 
	 * given a list of automata, find the optimal robot set - automaton pairs **/
	public List<RobotConfig> optListOfRobotConfig(Set<Character> CoopEventSet, 
			Set<List<Graph<RobotConfig,DefaultEdge>>> maxGraphSetOfRobotSet) 
	{
		List<Graph<RobotConfig, DefaultEdge>> optGraphOfRobotSet = new ArrayList<>();
		double[] Overallcoverage = new double[maxGraphSetOfRobotSet.size()];
		double MaxCoverage = 0;
		int index = 0;
		
		for (List<Graph<RobotConfig, DefaultEdge>> GraphOfRobotSet: maxGraphSetOfRobotSet) {
			
			for (Graph<RobotConfig, DefaultEdge> subGraphOfRobotSet : GraphOfRobotSet)
				for (RobotConfig robotConfig : subGraphOfRobotSet.vertexSet()) 
					Overallcoverage[index] += capabilityCoverage(robotConfig);
			
			if (Overallcoverage[index] > MaxCoverage) {
				MaxCoverage = Overallcoverage[index];
				optGraphOfRobotSet = GraphOfRobotSet;
			}
				
			index += 1;
		}
		
		return robotReconfig(optGraphOfRobotSet, CoopEventSet);
	}
	
	/** */
	public List<RobotConfig> robotReconfig(List<Graph<RobotConfig, DefaultEdge>> optGraphOfRobotSet, Set<Character> CoopEventSet) {
		
		List<RobotConfig> optRobotConfigList = new ArrayList<>();
		Set<String> CoopEventStringSet=  TaskAutomaton.charSet2StringSet(CoopEventSet);
		
		for (Graph<RobotConfig, DefaultEdge> subGraphOfRobotSet : optGraphOfRobotSet) {
			
			List<TaskAutomaton> automatonList = new ArrayList<>();
			Set<Robot> robotSets = new HashSet<>();
			for (RobotConfig robotConfig : subGraphOfRobotSet.vertexSet()) {
				automatonList.add(robotConfig.automaton);
				robotSets.addAll(robotConfig.robotSet);
			}
			
			TaskAutomaton composedAutomaton = automatonList.get(0);
			if (automatonList.size() > 1)
				for (int index = 1; index < automatonList.size(); index++) {
					composedAutomaton = TaskAutomaton.compositionOfTaskAut(composedAutomaton, automatonList.get(index), CoopEventStringSet);
				}
			
			optRobotConfigList.add(new RobotConfig(composedAutomaton, robotSets));
			System.out.println("Generated robot sets" + robotSets.size());
		}
		
		return optRobotConfigList;
	}
	
	/** Optimal robot configuration for automaton list: 
	 * given a list of automata, find the optimal robot set - automaton pairs **/
	public Set<List<Graph<RobotConfig, DefaultEdge>>> maxGraphSetOfRobotConfig(List<TaskAutomaton> automatonList, 
			Set<Robot> providedRobots, ArrayList<String> CoopEventSet) {
		
		Set<List<Graph<RobotConfig, DefaultEdge>>> SetOfStronglyConnectedSubgraphs = new HashSet<>();
		Set<List<Graph<RobotConfig, DefaultEdge>>> SetOfMaxIndepConnectedSubgraphs = new HashSet<>();
		int MAXSIZE = 0;
		
		Set<List<Set<Robot>>> robotsetSetLists = combinRobotsetLists(automatonList, providedRobots, CoopEventSet);
		//System.out.println("solution size are:" + robotsetSetLists.size());
		for (List<Set<Robot>> robotsetLists : robotsetSetLists) {
			//find the optimal one
			Graph<RobotConfig, DefaultEdge> switchGraphOfRobotset = robotsetSwitchRelation(automatonList, 
					robotsetLists);
			
			// computes all the strongly connected components of the directed graph
			BiconnectivityInspector<RobotConfig, DefaultEdge> scAlg =
					new BiconnectivityInspector<>(switchGraphOfRobotset);
			List<Graph<RobotConfig, DefaultEdge>> stronglyConnectedSubgraphs = new ArrayList<>(
					scAlg.getConnectedComponents());
			
			SetOfStronglyConnectedSubgraphs.add(stronglyConnectedSubgraphs);
			// output the strongly connected components
			/* Need to consider weight */
			if (stronglyConnectedSubgraphs.size() > MAXSIZE)
				MAXSIZE = stronglyConnectedSubgraphs.size();
		}
		
		for (List<Graph<RobotConfig, DefaultEdge>> ConnectedSubgraphs : SetOfStronglyConnectedSubgraphs)
			if (ConnectedSubgraphs.size() == MAXSIZE)
				SetOfMaxIndepConnectedSubgraphs.add(ConnectedSubgraphs);
		
		return SetOfMaxIndepConnectedSubgraphs;
	}
	
	/** 
	 * For new initialization equation *
	 * given a list of automata, find the optimal robot set - automaton pairs
	 * @param listOfMinRobotConfigSet
	 * @return Set of max indep connected subgraphs;
	 */
	 
	 public Set<List<Graph<RobotConfig, DefaultEdge>>> maxGraphSetOfRobotConfig(List<Set<RobotConfig>> listOfMinRobotConfigSet) {
		
		Set<List<Graph<RobotConfig, DefaultEdge>>> SetOfStronglyConnectedSubgraphs = new HashSet<>();
		Set<List<Graph<RobotConfig, DefaultEdge>>> SetOfMaxIndepConnectedSubgraphs = new HashSet<>();
		int MAXSIZE = 0;
		
//		Set<List<Set<Robot>>> robotsetSetLists = combinRobotsetLists(automatonList, providedRobots, CoopEventSet);
		Set<List<RobotConfig>> robotConfigTupleSet= SetOfCombiRobotConfigs(listOfMinRobotConfigSet);
		//System.out.println("solution size are:" + robotsetSetLists.size());
		for (List<RobotConfig> robotConfigTuple : robotConfigTupleSet) {
			//find the optimal one
			Graph<RobotConfig, DefaultEdge> switchGraphOfRobotset = relationOfRobotConfig(robotConfigTuple);
			
			// computes all the strongly connected components of the directed graph
			BiconnectivityInspector<RobotConfig, DefaultEdge> scAlg = new BiconnectivityInspector<>(switchGraphOfRobotset);
			List<Graph<RobotConfig, DefaultEdge>> stronglyConnectedSubgraphs = new ArrayList<>(scAlg.getConnectedComponents());
			
			SetOfStronglyConnectedSubgraphs.add(stronglyConnectedSubgraphs);
			// output the strongly connected components
			/* Need to consider weight */
			if (stronglyConnectedSubgraphs.size() > MAXSIZE)
				MAXSIZE = stronglyConnectedSubgraphs.size();
		}
		
		for (List<Graph<RobotConfig, DefaultEdge>> ConnectedSubgraphs : SetOfStronglyConnectedSubgraphs)
			if (ConnectedSubgraphs.size() == MAXSIZE)
				SetOfMaxIndepConnectedSubgraphs.add(ConnectedSubgraphs);
		
		return SetOfMaxIndepConnectedSubgraphs;
	}
	 
	/** Set of robot configuration set lists corresponding to each
	 * indexed automaton:
	 * given a list of automata and their corresponding minimal 
	 * robot configuration sets, obtain the Cartesian product of 
	 * all the robot configuration sets */
	public Set<List<Set<Robot>>> combinRobotsetLists (List<TaskAutomaton> automatonList, Set<Robot> providedRobots, 
			ArrayList<String> CoopEventSet) {
		
		List<Set<Set<Robot>>> ListOfMinRobotsets = new ArrayList<>();
		
		for (TaskAutomaton taskAutomaton : automatonList) {
			ListOfMinRobotsets.add(RobotConfig.minRobotConfigSet(taskAutomaton, providedRobots, CoopEventSet));
		}
		return Sets.cartesianProduct(ListOfMinRobotsets);
	}
	
	/** 
	 * For new initialization equation
	 * Set of robot configuration set lists corresponding to each 
	 * indexed automaton:
	 * given a list of automata and their corresponding minimal 
	 * robot configuration sets, obtain the Cartesian product of 
	 * all the robot configuration sets 
	 * @param ListOfMinRobotConfigSet
	 * @return set of combinatoral robot configurations  */
	
	public Set<List<RobotConfig>> SetOfCombiRobotConfigs (List<Set<RobotConfig>> ListOfMinRobotConfigSet) {
		
		return Sets.cartesianProduct(ListOfMinRobotConfigSet);
	}
	
	/** Graph of interdependency of each automaton - robot set pair:
	 * given a list of minimum configured robot sets corresponding 
	 * to their automaton index in the list, obtain the interdependency 
	 * relation between each two of these robot sets (automata)  */
	public Graph<RobotConfig, DefaultEdge> robotsetSwitchRelation(List<TaskAutomaton> automatonList, 
			List<Set<Robot>> robotSetLists) {
		
		Graph<RobotConfig, DefaultEdge> connectedRobotConfigGraph = new SimpleWeightedGraph<>(DefaultEdge.class);
		List<RobotConfig> RobotConfigList = new ArrayList<>();
		int index, index_;
		Set<Robot> intersection;
		
		for (index = 0; index < robotSetLists.size(); index++) {
			RobotConfig vertexOfRobotConfig = new RobotConfig(automatonList.get(index), robotSetLists.get(index));
			RobotConfigList.add(vertexOfRobotConfig);
			connectedRobotConfigGraph.addVertex(vertexOfRobotConfig);
		}
		
		for (index = 0; index < robotSetLists.size(); index++)
			for (index_ = index + 1; index_ < robotSetLists.size(); index_++)
			{
				intersection = new HashSet<>(RobotConfigList.get(index).robotSet);
				intersection.retainAll(RobotConfigList.get(index_).robotSet);
				
				if (!intersection.isEmpty()) {
					connectedRobotConfigGraph.addEdge(RobotConfigList.get(index), RobotConfigList.get(index_));
					connectedRobotConfigGraph.setEdgeWeight(RobotConfigList.get(index), 
							RobotConfigList.get(index_), intersection.size());
				}
			}
		
		return connectedRobotConfigGraph;
	}
	
	/** 
	 * For new initialization equation
	 * Graph of interdependency of each automaton - robot set pair:
	 * given a list of minimum configured robot sets corresponding 
	 * to their automaton index in the list, obtain the interdependency 
	 * relation between each two of these robot sets (automata)  
	 * @param robotConfigTuple
	 * @return a connected RobotConfig Graph*/
	public Graph<RobotConfig, DefaultEdge> relationOfRobotConfig(List<RobotConfig> robotConfigTuple) {
		
		Graph<RobotConfig, DefaultEdge> connectedRobotConfigGraph = new SimpleWeightedGraph<>(DefaultEdge.class);
		Set<Robot> intersection;
		
		for (RobotConfig robotConfig: robotConfigTuple) 
			connectedRobotConfigGraph.addVertex(robotConfig);
		
		for (RobotConfig vertex_s : connectedRobotConfigGraph.vertexSet())
			for (RobotConfig vertex_d : connectedRobotConfigGraph.vertexSet())
				if (!vertex_s.equals(vertex_d)) {
					intersection = new HashSet<>(vertex_s.robotSet);
					intersection.retainAll(vertex_d.robotSet);
					
					if (!intersection.isEmpty()) {
						connectedRobotConfigGraph.addEdge(vertex_s, vertex_d);
						connectedRobotConfigGraph.setEdgeWeight(vertex_s, vertex_d, intersection.size());
					}
				}
		
		return connectedRobotConfigGraph;
	}
	
	/**
	 * Desaribe the coverage of a set of robot capability 
	 * @param robotConfig 
	 * @return a coverage value*/
	private double capabilityCoverage(RobotConfig robotConfig) {
		double coverage;
		List<String> overallCapability = new ArrayList<String>();
		Set<String> automatonEventSet;
		
		for (Robot robot : robotConfig.robotSet)
			overallCapability.addAll(robot.eventList());
		
		automatonEventSet = robotConfig.automaton.eventSet;
		overallCapability.retainAll(automatonEventSet);
		coverage = overallCapability.size() / automatonEventSet.size();
		
		return coverage;
	}
	
}
