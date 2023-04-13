package taskPlanning1;

import java.util.PriorityQueue;

import dk.brics.automaton.State;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;

public class DijkstraAlgo {
	/* Dijkstra Algorithm
	 * 
	 *
	 */
	public static void computePaths(Node source){
		source.shortestDistance=0;

		//implement a priority queue
		PriorityQueue<Node> queue = new PriorityQueue<Node>();
		queue.add(source);

		while(!queue.isEmpty()){
			Node u = queue.poll();

			/*visit the adjacencies, starting from 
			the nearest node(smallest shortestDistance)*/
			
			for(Edge e: u.adjacencies){

				Node v = e.target;
				double weight = e.weight;

				//relax(u,v,weight)
				double distanceFromU = u.shortestDistance+weight;
				if(distanceFromU<v.shortestDistance){

					/*remove v from queue for updating 
					the shortestDistance value*/
					queue.remove(v);
					v.shortestDistance = distanceFromU;
					v.parent = u;
					v.parentEdge = e;
					queue.add(v);

				}
			}
		}
	}

	public static List<Node> getShortestPathTo(Node target){

		//trace path from target to source
		List<Node> path = new ArrayList<Node>();
		for(Node node = target; node!=null; node = node.parent){
			path.add(node);
		}


		//reverse the order such that it will be from source to target
		Collections.reverse(path);

		return path;
	}
	
	public static List<Character> getShortestActPathTo(Node target){

		//trace path from target to source
		List<Character> path = new ArrayList<Character>();
		for(Node node = target; node.parentEdge!=null; node = node.parent){
			path.add(node.parentEdge.action);
		}


		//reverse the order such that it will be from source to target
		Collections.reverse(path);

		return path;
	}

}

//define Node
class Node implements Comparable<Node>{
	
	public final String value;
	public List<Edge> adjacencies;
	public double shortestDistance = Double.POSITIVE_INFINITY;
	public Node parent;
	public Edge parentEdge;

	public Node(String val){
		value = val;
	}
	
	public Node(State stat){
		value = stat.toString();
	}

	public String toString(){
			return value;
	}

	public int compareTo(Node other){
		return Double.compare(shortestDistance, other.shortestDistance);
	}

}

//define Edge
class Edge{
	public final Node target;
	public final double weight;
	public Character action;
	
	public Edge(Node targetNode, double weightVal){
		target = targetNode;
		weight = weightVal;
	}	

	public Edge(Node targetNode, double weightVal, Character actLabel){
		target = targetNode;
		weight = weightVal;
		action = actLabel;
	}
}
