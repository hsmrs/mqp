/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	RobotListModel.java												***
***		This class models a list of robotic agents.	It is an		***
***		implementation of the AbstractListModel.					***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model.robot;

import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractListModel;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;

public class RobotListModel extends AbstractListModel{

	private static RobotListModel instance;
	private List<RobotModel> robotList;
	
	/**
	 * The constructor for the RobotListModel. It is private because this class is a Singleton.
	 */
	private RobotListModel()
	{
		robotList = new ArrayList<RobotModel>();
		
	}
	
	/**
	 * Returns the instance of the only existing RobotListModel.
	 * @return The instance of the only existing RobotListModel.
	 */
	static public synchronized RobotListModel getInstance() {
		if (instance == null)
			instance = new RobotListModel();
		return instance;
	}
	
	/**
	 * Adds a given RobotModel to the list of robots in the system.
	 * @param newRobot The RobotModel to be added.
	 */
	public void addRobot(RobotModel newRobot){
		robotList.add(newRobot);
		int index = robotList.size() - 1;
		this.fireIntervalAdded(this, index, index);
		ConsoleController.getInstance().addConsoleChannel(newRobot.getName());
	}
	
	/**
	 * Removes the given robot from the list of robots in the system.
	 * @param targetRobot The RobotModel to be removed from the list.
	 * @return
	 */
	public RobotModel removeRobot(RobotModel targetRobot){
		int index = robotList.indexOf(targetRobot);
		robotList.remove(targetRobot);
		this.fireIntervalRemoved(this, index, index);
		ConsoleController.getInstance().removeConsoleChannel(targetRobot.getName());
		return targetRobot;
	}
	
	/**
	 * Removes the RobotModel from the list of robots in the system which is located at the
	 * given index
	 * @param index The index of the robot to be removed from the list.
	 * @return
	 */
	public RobotModel removeRobot(int index){
		RobotModel targetRobot = getElementAt(index);
		robotList.remove(index);
		this.fireIntervalRemoved(this, 0, 0);
		ConsoleController.getInstance().removeConsoleChannel(targetRobot.getName());
		return targetRobot;
	}
	
	/**
	 * Returns the robot at the given index.
	 * @return The robot at the given index.
	 */
	public RobotModel getElementAt(int index) {
		return robotList.get(index);
	}

	/**
	 * Returns the number of robots in the system.
	 */
	public int getSize() {
		return robotList.size();
	}
	
	/**
	 * Returns the list of robots in the system.
	 * @return The list of robots in the system.
	 */
	public List<RobotModel> getRobots(){
		return robotList;
	}
	
	/**
	 * Returns the list of robots in the system by name.
	 * @return The list of robots in the system by name.
	 */
	public List<String> getRobotNames(){
		ArrayList<String> robotNames = new ArrayList<String>(robotList.size());
		for (RobotModel robot : robotList){
			robotNames.add(robot.getName());
		}
		return robotNames;
	}
	
	/**
	 * Returns the RobotModel which has the given name.
	 * @param name The name of the desired RobotModel
	 * @return The RobotModel with the given name.
	 */
	public RobotModel getRobotModelByName(String name){
		for (RobotModel rm : robotList){
			if (rm.getName().equals(name)){
				return rm;
			}
		}
		return null;
	}
	
}
