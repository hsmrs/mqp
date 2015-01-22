package src.main.java.com.github.hsmrs_gui.project.controller;

import javax.swing.JOptionPane;

import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotListView;

public class RobotController {

	private static RobotController instance;
	private RobotListView robotListView;

	private RobotController() {
	}

	public static synchronized RobotController getInstance() {
		if (instance == null) {
			instance = new RobotController();
		}

		return instance;
	}

	public void setRobotListView(RobotListView view) {
		robotListView = view;
	}

	public void setRobotStatus(String robotName, String robotStatus) {
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setStatus(robotStatus);
		robotListView.update();
	}

	public void setRobotNeedsHelp(String robotName, boolean needsHelp) {
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(
				robotName);
		robot.setNeedsHelp(needsHelp);
		robotListView.update();

		if (needsHelp) {
			JOptionPane.showMessageDialog(robotListView, robot.getName() + " has requested assistance!",
					"Attention Required", JOptionPane.WARNING_MESSAGE);
		}
	}
}
