/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	RobotListModel.java												***
***		This class models a list of robotic agents.	It is an		***
***		implementation of the AbstractListModel.					***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model;

import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractListModel;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;

public class RobotListModel extends AbstractListModel{

	private static RobotListModel instance;
	private List<RobotModel> robotList;
	
	private RobotListModel()
	{
		robotList = new ArrayList<RobotModel>();
		
	}
	
	static public synchronized RobotListModel getInstance() {
		if (instance == null)
			instance = new RobotListModel();
		return instance;
	}
	
	public void addRobot(RobotModel newRobot){
		robotList.add(newRobot);
		int index = robotList.size() - 1;
		this.fireIntervalAdded(this, index, index);
		ConsoleController.getInstance().addConsoleChannel(newRobot.getName());
	}
	
	public RobotModel removeRobot(RobotModel targetRobot){
		int index = robotList.indexOf(targetRobot);
		robotList.remove(targetRobot);
		this.fireIntervalRemoved(this, index, index);
		ConsoleController.getInstance().removeConsoleChannel(targetRobot.getName());
		return targetRobot;
	}
	
	public RobotModel removeRobot(int index){
		RobotModel targetRobot = getElementAt(index);
		robotList.remove(index);
		this.fireIntervalRemoved(this, 0, 0);
		ConsoleController.getInstance().removeConsoleChannel(targetRobot.getName());
		return targetRobot;
	}
	
	public RobotModel getElementAt(int index) {
		return robotList.get(index);
	}

	public int getSize() {
		return robotList.size();
	}
	
	public List<String> getRobotNames(){
		ArrayList<String> robotNames = new ArrayList<String>(robotList.size());
		for (RobotModel robot : robotList){
			robotNames.add(robot.getName());
		}
		return robotNames;
	}
	
	public RobotModel getRobotModelByName(String name){
		for (RobotModel rm : robotList){
			if (rm.getName().equals(name)){
				return rm;
			}
		}
		return null;
	}
	
}
