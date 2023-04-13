package taskPlanning2;

/** Define the robot action class */
public class RobotAction {
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