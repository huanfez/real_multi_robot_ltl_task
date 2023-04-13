package taskPlanning2;

import java.util.ArrayList;
import java.util.List;

/** Define the class of product automaton vertex*/
public class VtxOfProdAut{
	ArrayList<String> TsState = new ArrayList<>();
	String AutState;
	List<Boolean> robotState = new ArrayList<>();
	
	public VtxOfProdAut() {};
	public VtxOfProdAut(ArrayList<String> TsState, String AutState){
		this.TsState = TsState;
		this.AutState = AutState;
	}
	
}