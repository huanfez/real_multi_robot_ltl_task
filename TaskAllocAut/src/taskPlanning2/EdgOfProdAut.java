package taskPlanning2;

import java.util.ArrayList;

/** Define the class of product automaton edge */
public class EdgOfProdAut {
	ArrayList<Integer> robotID;
	String action;
	double weight;
	
	public EdgOfProdAut() {
		robotID = new ArrayList<Integer>();
	};
	
	public EdgOfProdAut(String action, ArrayList<Integer> robotID){
		this.action = action;
		this.robotID = robotID;
	}
	
}