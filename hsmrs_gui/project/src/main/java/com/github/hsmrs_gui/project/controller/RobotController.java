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
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotListView;
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotPanel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskPanel;

public class RobotController implements ActionListener{

	private static RobotController instance;
	private RobotPanel robotView;

	private RobotController() {
	}

	public static synchronized RobotController getInstance() {
		if (instance == null) {
			instance = new RobotController();
		}

		return instance;
	}

	public void setRobotView(RobotPanel view) {
		robotView = view;
	}

	public void setRobotStatus(String robotName, String robotStatus) {
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setStatus(robotStatus);
		robotView.updateRobotList();;
	}

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

	public void setRole(String robotName, RoleModel role){
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setRole(role);
		robotView.updateRobotList();
	}
	
	public void removeRole(String robotName) {
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setRole(null);
		robotView.updateRobotList();		
	}

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
