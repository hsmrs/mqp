package src.main.java.com.github.hsmrs_gui.project.controller;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JOptionPane;

import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RoleListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RoleModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskParam;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.ros.NewTaskPublisher;
import src.main.java.com.github.hsmrs_gui.project.ros.RolePublisher;
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotListView;
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotPanel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskPanel;

public class RobotController implements ActionListener{

	private static RobotController instance;
	private RobotPanel robotView;

	/**
	 * This is the constructor for the RobotController class.
	 */
	private RobotController() {
	}

	/**
	 * Returns the only existing instance of the Singleton RobotController class.
	 * @return the instance of the RobotController class
	 */
	public static synchronized RobotController getInstance() {
		if (instance == null) {
			instance = new RobotController();
		}

		return instance;
	}

	/**
	 * Sets the view that the controller will use for its actions
	 * @param view The RobotPanel that this controller will work with.
	 */
	public void setRobotView(RobotPanel view) {
		robotView = view;
	}

	/**
	 * Modify the robot model and view to reflect the status of the named robot.
	 * @param robotName The robot whose status will be changed.
	 * @param robotStatus The new status for the robot.
	 */
	public void setRobotStatus(String robotName, String robotStatus) {
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setStatus(robotStatus);
		robotView.updateRobotList();;
	}

	/**
	 * Sets the help state of the robot. If the robot needs help, this creates a message dialog which will
	 * get the user's attention.
	 * @param robotName The robot whose help state needs to be changed.
	 * @param needsHelp True if the robot needs help, false otherwise.
	 */
	public void setRobotNeedsHelp(String robotName, boolean needsHelp) {
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setNeedsHelp(needsHelp);
		robotView.updateRobotList();

		if (needsHelp) {
			JOptionPane.showMessageDialog(robotView, robot.getName() + " has requested assistance!",
					"Attention Required", JOptionPane.WARNING_MESSAGE);
		}
	}

	/**
	 * Gives the named robot the given role and updates the view.
	 * @param robotName The name of the robot who will receive the role.
	 * @param role The role to be given to the robot.
	 */
	public void setRole(String robotName, RoleModel role){
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setRole(role);
		robotView.updateRobotList();
	}
	
	/**
	 * Removes the role from the named robot.
	 * @param robotName The name of the robot whose role will be removed.
	 */
	public void removeRole(String robotName) {
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setRole(null);
		robotView.updateRobotList();		
	}

	/**
	 * This method is called whenever an action which has been linked to this controller is
	 * executed. This method determines what type of action has occurred and responds appropriately.
	 */
	@Override
	public void actionPerformed(ActionEvent e) {
		String cmd = e.getActionCommand();
		RobotPanel rp = RobotPanel.getInstance();
		
		if (cmd.equals("New Role")){
			RobotPanel.getInstance().setView(RobotPanel.NEW_ROLE_VIEW);
		}
		else if (cmd.equals("Assign Role")){
			RoleModel assignedRole = rp.getSelectedRole();
			RobotModel robot = RobotPanel.getInstance().getTargetRobot();			
			robot.setRole(assignedRole);
			RolePublisher.getInstance().publishRole(assignedRole, robot);
			System.out.println("Setting role to " + robot.getName());
			rp.setView(RobotPanel.ROBOT_LIST_VIEW);
			rp.updateRobotList();
		}
		else if (cmd.equals("Cancel Role")){
			rp.setView(RobotPanel.ROBOT_LIST_VIEW);
		}
		else if (cmd.equals("Create New Role")){
			RoleModel newRole = rp.getNewRole();
			RoleListModel.getInstance().addRole(newRole);
			rp.setView(RobotPanel.ROLE_LIST_VIEW);
		}
		else if (cmd.equals("Cancel New Role")){
			rp.switchView(RobotPanel.ROLE_LIST_VIEW);
		}
	}
}
