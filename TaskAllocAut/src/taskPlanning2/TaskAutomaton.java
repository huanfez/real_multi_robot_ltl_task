package taskPlanning2;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DirectedPseudograph;

import dk.brics.automaton.Automaton;
import dk.brics.automaton.State;
import dk.brics.automaton.Transition;
import dk.brics.automaton.decompAut;

public class TaskAutomaton {
	
	public String initialState;
	public ArrayList<String> acceptedStates = new ArrayList<>();
	public DirectedPseudograph<String,AutomatonEdge> AutomatonGraph;
	public Set<String> eventSet = new HashSet<>();
	
	public TaskAutomaton() {};
	
	public TaskAutomaton(String initialState, ArrayList<String> acceptedStates, DirectedPseudograph<String, 
			AutomatonEdge> AutomatonGraph, Set<String> eventSet) {
		this.initialState = initialState;
		this.acceptedStates = acceptedStates;
		this.AutomatonGraph = AutomatonGraph;
		this.eventSet = eventSet;
	}
	
	/** Initialization: task automaton described by new class**********/
	public TaskAutomaton(Automaton taskAutomaton) {
		
		System.out.println("Original automaton" + taskAutomaton);
		
		DirectedPseudograph<String,AutomatonEdge> taskAutomatonGraph = new DirectedPseudograph<>(
				AutomatonEdge.class);
		String autoStateInx, autoStateInx_;
		
		Set<State> autoStateSet= taskAutomaton.getStates();
		
		/* * Convert automaton states into graph vertices**/
		for (State autoState : autoStateSet) {
			autoStateInx = extractInteger(autoState.toString());
			taskAutomatonGraph.addVertex(autoStateInx);
		}
		
		for (State autoState : autoStateSet) {
			autoStateInx = extractInteger(autoState.toString());
			
			for (Transition transition : autoState.getTransitions()) {
				autoStateInx_ = extractInteger(transition.getDest().toString());
				
				for (int event = (int)transition.getMin(); event <= (int)transition.getMax(); event++) {
					AutomatonEdge edge = new AutomatonEdge(Character.toString((char)event));
					taskAutomatonGraph.addEdge(autoStateInx, autoStateInx_, edge);
				}
			}
		}
		
		/* * Add initial vertex for automaton graph**/
		this.initialState = extractInteger(taskAutomaton.getInitialState().toString());
		
		/* * Add accepted vertices for automaton graph**/
		for (State state : taskAutomaton.getAcceptStates())
			this.acceptedStates.add(extractInteger(state.toString()));
		
		this.AutomatonGraph = taskAutomatonGraph;
		
		this.eventSet = charSet2String(taskAutomaton);
		
		System.out.println("The converted task automaton init state:" + this.initialState);
		System.out.println("The converted task automaton graph:" + this.AutomatonGraph);
	}
	
	/** Task automaton described in graph**********/
	public static TaskAutomaton compositionOfTaskAut(TaskAutomaton taskAutomaton1, TaskAutomaton taskAutomaton2, 
			Set<String> cooperativeEventSet) {
		//System.out.println("Original automaton" + taskAutomaton);
		
		TaskAutomaton composedAutomaton = new TaskAutomaton();
		DirectedPseudograph<String,AutomatonEdge> composedAutomatonGraph = new DirectedPseudograph<>(AutomatonEdge.class);
		String composedInitState = "";
		ArrayList<String> composedAcceptedStateSet = new ArrayList<>();
		Set<String> compositeVertexSet  = new HashSet<>();
		
		DirectedPseudograph<String,AutomatonEdge> TA1Graph = taskAutomaton1.AutomatonGraph;
		DirectedPseudograph<String,AutomatonEdge> TA2Graph = taskAutomaton2.AutomatonGraph;
		
		Set<String> TA1VertexSet = TA1Graph.vertexSet();
		Set<String> TA2VertexSet = TA2Graph.vertexSet();
		
		for (String TA1Vertex : TA1VertexSet)
			for (String TA2Vertex : TA2VertexSet){
				String compositeVertex = TA1Vertex + ',' + TA2Vertex;
				composedAutomatonGraph.addVertex(compositeVertex);
				
				if (TA1Vertex.equals(taskAutomaton1.initialState) && TA2Vertex.equals(taskAutomaton2.initialState))
					composedInitState = compositeVertex;
				
				if (taskAutomaton1.acceptedStates.contains(TA1Vertex) && taskAutomaton2.acceptedStates.contains(TA2Vertex))
					composedAcceptedStateSet.add(compositeVertex);
			}
		
		composedAutomaton.initialState = composedInitState;
		composedAutomaton.acceptedStates = composedAcceptedStateSet;
		
		compositeVertexSet = composedAutomatonGraph.vertexSet();
		for (String compositeVertexS : compositeVertexSet){
			
			int beginIndexS = compositeVertexS.lastIndexOf(',');
			String cVertexS1 = compositeVertexS.substring(0, beginIndexS);
			String cVertexS2 = compositeVertexS.substring(beginIndexS+1);
			
			for (String compositeVertexD : compositeVertexSet){
				
				int beginIndexD = compositeVertexD.lastIndexOf(',');
				String cVertexD1 = compositeVertexD.substring(0, beginIndexD);
				String cVertexD2 = compositeVertexD.substring(beginIndexD+1);
				
				AutomatonEdge cVertexSD1 = TA1Graph.getEdge(cVertexS1, cVertexD1);
				
				AutomatonEdge cVertexSD2 = TA2Graph.getEdge(cVertexS2, cVertexD2);
				
				if (cVertexSD1 != null && cVertexSD2 != null && cVertexSD1.Event.equals(cVertexSD2.Event)){
					composedAutomatonGraph.addEdge(compositeVertexS, compositeVertexD, new AutomatonEdge(cVertexSD1.Event));
					System.out.print("Task automaton 1:" + cVertexS1 + cVertexSD1.Event + cVertexD1);
					System.out.println("Task automaton 2:" + cVertexS2 + cVertexSD2.Event + cVertexD2);
				}
				else if (cVertexSD1 != null && cVertexS2.equals(cVertexD2) && !cooperativeEventSet.contains(cVertexSD1.Event)){
					composedAutomatonGraph.addEdge(compositeVertexS, compositeVertexD, new AutomatonEdge(cVertexSD1.Event));
					System.out.println("Task automaton 1:" + cVertexS1 + cVertexSD1.Event + cVertexD1);
				}
				else if (cVertexS1.equals(cVertexD1) && cVertexSD2 != null && !cooperativeEventSet.contains(cVertexSD2.Event)){
					composedAutomatonGraph.addEdge(compositeVertexS, compositeVertexD, new AutomatonEdge(cVertexSD2.Event));
					System.out.println("Task automaton 2:" + cVertexS2 + cVertexSD2.Event + cVertexD2);
				}
			}
		}
		
		composedAutomaton.AutomatonGraph = composedAutomatonGraph;
		
		return composedAutomaton;
	}
	
	/** Extract first occurrence of integer from string*****/
	public static String extractInteger(String state) {
		Pattern p = Pattern.compile("([\\d]+)");
		Matcher m = p.matcher(state);
		if (m.find()) {
			return m.group(0);
		}
		
		return null;
	}
	
	/** String format of event set of automaton*/
	public static Set<String> charSet2String(Automaton automaton) {
		Set<String> eventSetOfAutInString = new HashSet<>();
		Set<Character> eventSetOfAut = decompAut.getEventSet(automaton);
		
		for (Character event: eventSetOfAut)
			eventSetOfAutInString.add(Character.toString(event));
		
		return eventSetOfAutInString;
	}
	
	/** String format of event List */
	public static ArrayList<String> charSet2StringList(Set<Character> eventSet) {
		ArrayList<String> eventSetOfAutInString = new ArrayList<>();
		
		for (Character event: eventSet)
			eventSetOfAutInString.add(Character.toString(event));
		
		return eventSetOfAutInString;
	}
	
	/** String format of event set */
	public static Set<String> charSet2StringSet(Set<Character> eventSet) {
		Set<String> eventSetOfAutInString = new HashSet<>();
		
		for (Character event: eventSet)
			eventSetOfAutInString.add(Character.toString(event));
		
		return eventSetOfAutInString;
	}
}


/** Define the labeled edge, allowing same name */
class AutomatonEdge 
	extends
	DefaultEdge	{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	public String Event;
	
	public AutomatonEdge(String automatonEvent) {
		this.Event = automatonEvent;
	}
}