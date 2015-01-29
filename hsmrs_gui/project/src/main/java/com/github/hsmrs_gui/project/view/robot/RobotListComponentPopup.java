package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPopupMenu;

import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;

public class RobotListComponentPopup extends JPopupMenu implements ActionListener{
	
	private RobotModel robot;
	private JComponent parent;

	public RobotListComponentPopup(RobotModel robot, JComponent parent){
		this.robot = robot;
		this.parent = parent;
		
		JMenuItem itemAddOwner = new JMenuItem("Set Role");
		itemAddOwner.addActionListener(this);
		JMenuItem itemRemoveOwner = new JMenuItem("Remove Role");
		itemRemoveOwner.addActionListener(this);
		JMenuItem itemTeleOp = new JMenuItem((robot.getStatus().equals("Tele-Op") ? "Stop Tele-Op" : "Tele-Op"));
		itemTeleOp.addActionListener(this);
//		JMenuItem itemRemove = new JMenuItem("Remove");
//		itemRemove.addActionListener(this);
		add(itemAddOwner);
		add(itemRemoveOwner);
		add(itemTeleOp);
		//add(itemRemove);
	}
	
	public void actionPerformed(ActionEvent e) {
		JMenuItem item = (JMenuItem) e.getSource();

		if (item.getText().equals("Set Role")) {
			RobotPanel.getInstance().setView(RobotPanel.ROLE_LIST_VIEW);
			RobotPanel.getInstance().setTargetRobot(robot);
		} else if (item.getText().equals("Remove Role")) {
			RobotController.getInstance().removeRole(robot.getName());
		} else if (item.getText().equals("Tele-Op")) {
			robot.getRequestPublisher().publishTeleOpRequest();
		}else if (item.getText().equals("Stop Tele-Op")) {
			robot.getRequestPublisher().publishStopTeleOpRequest();
		}/*else if (item.getText().equals("Remove")) {
			TaskListModel.getInstance().removeTask(task);
			return;
		}*/
	}
	
}
