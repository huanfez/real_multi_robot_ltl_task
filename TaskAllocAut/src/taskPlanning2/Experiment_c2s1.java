package taskPlanning2;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.jgrapht.GraphPath;
import org.jgrapht.io.ExportException;

import dk.brics.automaton.Automaton;
import dk.brics.automaton.BasicOperations;
import dk.brics.automaton.RegExp;
import dk.brics.automaton.SubAutomataExtract;
import dk.brics.automaton.decompAut;

public class Experiment_c2s1 {
	
	/** Robot initial state of transition system*/
	private static ArrayList<String> initNode0 = new ArrayList<String>(Arrays.asList("a"));

	private static ArrayList<String> initNode1 = new ArrayList<String>(Arrays.asList("a"));

	private static ArrayList<String> initNode2 = new ArrayList<String>(Arrays.asList("i"));

	private static ArrayList<String> initNode3 = new ArrayList<String>(Arrays.asList("b"));
	
	/** Main function **/
	/** ideal input: 
	 *  (1. Automaton(R.E.) described task specification, 
	 *  2. Cooperative event set, 
	 *  3. Initial State of Robot) */
	public static void main(String args[]) throws ExportException, IOException {
		
		////////////////////////////Initial Decomposition and Allocation///////////////////////////////
		/** 0. Definition of robots and their transition systems **/		
		Robot robot0 = new Robot(Experiment.robot0ActionList, Experiment.ts0Graph, initNode0);
		Robot robot1 = new Robot(Experiment.robot1ActionList, Experiment.ts1Graph, initNode1);
		Robot robot2 = new Robot(Experiment.robot2ActionList, Experiment.ts2Graph, initNode2);
		Robot robot3 = new Robot(Experiment.robot3ActionList, Experiment.ts3Graph, initNode3);
		Set<Robot> providedRobots = new HashSet<>(Arrays.asList(robot0, robot1 ,robot2, robot3));
		
		/** 1.1 give a set of regular expressions, output automata*/
//		final RegExp regExpr_1 = new RegExp("b((dc)|(cd))((ef)|(fe))");
//		final RegExp regExpr_2 = new RegExp("c((bf)|(fb))((de)|(ed))");
//		final RegExp regExpr_3 = new RegExp("((bc)|(cb))e((df)|(fd))");
//		final RegExp regExpr_4 = new RegExp("(((bc)|(cb))((ed)|(de)))|((bd)|((ce)|(ec(f|e))))|(be((((cd)|(dc))(e|f))|(c(e|f)a)))");
		final RegExp regExpr_4 = new RegExp("(((ba)|(ab))((d((ef)|(fe)))|(e((df)|(fd)))|(f((de)|(ed)))))|(bd((eaf)|(aef)|(afe)))|(be((daf)|(adf)|(afd)))|(afb((de)|(ed)))");
		
		Set<Automaton> originalAutomata = new HashSet<Automaton>() {/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

		{
//			add(regExpr_1.toAutomaton());
//			add(regExpr_2.toAutomaton());
//			add(regExpr_3.toAutomaton());
			add(regExpr_4.toAutomaton());
		}};
		
		Automaton globalTaskAutomaton = BasicOperations.union(originalAutomata);
		
		/** 1.2 Declare cooperative events **/
		Set<Character> CoopEventSet = new HashSet<Character>() {/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

		{
			add('g');
		}};
		
		/** 2.1. Obtain sub-automaton, of which all paths have common events **/
		Set<Automaton> SubautomatonSet= SubAutomataExtract.commonEventsAut(globalTaskAutomaton);
		
		/** 2.2. Generate parallel decompositions for each sub-automaton**/
		List<List<Automaton>> decompositionSet = new ArrayList<List<Automaton>>();
		for (Automaton automaton : SubautomatonSet)
			decompositionSet.add(decompAut.paraDecompC(automaton, CoopEventSet));
		
		/** 2.3. Convert each decomposition into graph form**/
		List<List<TaskAutomaton>> decompositionSetConvert = new ArrayList<List<TaskAutomaton>>();
		for (List<Automaton> decompositions : decompositionSet) {
			
			List<TaskAutomaton> decompositionsConvert = new ArrayList<TaskAutomaton>();
			for (Automaton singleAutomaton : decompositions) {
				decompositionsConvert.add(new TaskAutomaton(singleAutomaton));
			}
			
			decompositionSetConvert.add(decompositionsConvert);
		}
		
		
		////////////////////////////////////Manually configure the robot set to each automaton/////////////////////////////////////////////////////////////////
		/** 3. Obtain automaton decompositions - robots configuration **/
		List<RobotSetSwitch> robotSetSwitchList = new ArrayList<>();
//		for (int index = 0; index < decompositionSetConvert.size(); index++)
//		robotSetSwitchList.add(new RobotSetSwitch(decompositionSetConvert.get(0), CoopEventSet, providedRobots));
		
		RobotConfig robotConfig01 = new RobotConfig(decompositionSetConvert.get(0).get(0), new HashSet<Robot>(Arrays.asList(robot0, robot2)));
		RobotConfig robotConfig02 = new RobotConfig(decompositionSetConvert.get(0).get(0), new HashSet<Robot>(Arrays.asList(robot1, robot2)));
		RobotConfig robotConfig11 = new RobotConfig(decompositionSetConvert.get(0).get(1), new HashSet<Robot>(Arrays.asList(robot2)));
		RobotConfig robotConfig12 = new RobotConfig(decompositionSetConvert.get(0).get(1), new HashSet<Robot>(Arrays.asList(robot3)));
		Set<RobotConfig>setOfRobotConfigs_0  = new HashSet<>(Arrays.asList(robotConfig01, robotConfig02));
		Set<RobotConfig>setOfRobotConfigs_1  = new HashSet<>(Arrays.asList(robotConfig11, robotConfig12));
		List<Set<RobotConfig>> ListOfMinRobotConfigSet = new ArrayList<>(Arrays.asList(setOfRobotConfigs_0, setOfRobotConfigs_1));
		robotSetSwitchList.add(new RobotSetSwitch(ListOfMinRobotConfigSet, CoopEventSet));
//		System.out.println(robotSwitchConfigs.get(0).autRobotsPair.size());
		
		/** 4. For each decomposition set, take product automaton with transition system to 
		/* get the subtask allocation automaton **/
		List<List<TaskPlanningAutomaton>> ListOfsubtaskAllocAutomatonLists = new ArrayList<>();
		List<List<GraphPath<VtxOfProdAut,EdgOfProdAut>>> ListOfShortestPathOfSubAllocationLists=new ArrayList<>();
		
		for (RobotSetSwitch robotSetConfig : robotSetSwitchList) {
			List<TaskPlanningAutomaton> subtaskAllocAutomatonList = new ArrayList<TaskPlanningAutomaton>();
			List<GraphPath<VtxOfProdAut, EdgOfProdAut>> shortestPathList = new ArrayList<>();
			
			for (RobotConfig singleConfig : robotSetConfig.autRobotsPair) {
				System.out.println("---------------------------");
				System.out.println("Subtask automaton is:" + singleConfig.automaton.AutomatonGraph.vertexSet());
				
				RobotTransitionSystem composedTS = RobotTransitionSystem.transitionSystemComposition(	singleConfig.robotSet, 
						TaskAutomaton.charSet2StringList(CoopEventSet));
				TaskPlanningAutomaton subtaskAllocAutomatonT = new TaskPlanningAutomaton(composedTS, 	singleConfig.automaton, providedRobots);
				subtaskAllocAutomatonList.add(subtaskAllocAutomatonT);
				
				//System.out.println("Subtask allocation automaton vertex set is:");
				//for (VtxOfProdAut vortex : subtaskAllocAutomatonT.taskAllocAutGraph.vertexSet())
					//System.out.println(vortex.TsState + "," + vortex.AutState + "," + vortex.robotState + " ");
				String fileName = "SPA" + ListOfsubtaskAllocAutomatonLists.size()
					+ subtaskAllocAutomatonList.size();
				TaskPlanningAutomaton.renderProdAutGraph(subtaskAllocAutomatonT.taskAllocAutGraph, fileName);
				
				/** 4.2 optimal path search**/
				GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath = subtaskAllocAutomatonT.lowestCostPath();
				String fileNameOfPath = "opt" + fileName;				
				TaskPlanningAutomaton.outputSuboptimalPath(shortestPath, fileNameOfPath);
				
				shortestPathList.add(shortestPath);
			}
			
			ListOfShortestPathOfSubAllocationLists.add(shortestPathList);
			ListOfsubtaskAllocAutomatonLists.add(subtaskAllocAutomatonList);
		}
		
		TaskPlanningAutomaton.outputFinalOptimalPath(ListOfShortestPathOfSubAllocationLists,  "FinalOptimalPath");
	}
}
