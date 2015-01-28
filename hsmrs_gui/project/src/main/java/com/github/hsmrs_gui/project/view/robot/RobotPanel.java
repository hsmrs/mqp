package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.Color;
import java.awt.GridLayout;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JPanel;
import javax.swing.ListModel;

import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RoleModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.view.task.NewTaskPanel;
import src.main.java.com.github.hsmrs_gui.project.view.task.NewTaskTypePanel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskListPanel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskPanel;

public class RobotPanel extends JPanel{
	
	private static RobotPanel instance;
	private RobotListView robotListView;
	private RoleListPanel roleListPanel;
	private NewRolePanel newRolePanel;
	private RobotModel targetRobot;
	
	public static final int ROBOT_LIST_VIEW = 0;
	public static final int ROLE_LIST_VIEW = 1;
	public static final int NEW_ROLE_VIEW = 2;
	
	private RobotPanel(){
		this.setBorder(BorderFactory.createMatteBorder(1, 1, 3, 3, Color.black));
		//this.setLayout(new MigLayout("insets 0, top, fill, debug", "[]", "[]"));
		this.setLayout(new GridLayout());
		robotListView = RobotListView.getInstance();
		RobotController.getInstance().setRobotView(this);
		add(robotListView);
	}
	
	public static RobotPanel getInstance(){
		if (instance == null){
			instance = new RobotPanel();
		}
		
		return instance;
	}
	
	public void setView(int viewCode){
		removeAll();
		validate();
		
		switch (viewCode){
		case ROBOT_LIST_VIEW:
			add(robotListView);//, BorderLayout.CENTER);
			//add(taskListView, "push, grow, top");
			break;
		case ROLE_LIST_VIEW:
			roleListPanel = new RoleListPanel();
			add(roleListPanel, "push, grow, top");
			break;
		case NEW_ROLE_VIEW:
			newRolePanel = new NewRolePanel();
			add(newRolePanel, "push, grow, top");
			break;
		}
		
		//add(new JLabel("I HATE THIS"));
		
		validate();
		repaint();
	}
	
	public void updateRobotList(){
		robotListView.update();
	}
	
	public void switchView(int viewCode) {
		removeAll();
		
		switch (viewCode){
		case ROBOT_LIST_VIEW:
			add(robotListView, "push, grow, top");
			break;
		case ROLE_LIST_VIEW:
			add(roleListPanel, "push, grow, top");
			break;
		case NEW_ROLE_VIEW:
			add(newRolePanel, "push, grow, top");
			break;
		}
		revalidate();
		
		repaint();
	}
	
	public void setListModel(ListModel rlm){
		robotListView.setListModel(rlm);
	}

	public RoleModel getSelectedRole() {
		return roleListPanel.getSelectedRole();
	}

	public RobotModel getTargetRobot() {
		return targetRobot;
	}

	public void setTargetRobot(RobotModel robot) {
		targetRobot = robot;		
	}

	public RoleModel getNewRole() {
		return newRolePanel.getNewRole();
	}

	
	/*public List<TaskModel> getSelectedTask(){
		return taskListView.getSelectedTask();
	}
	
	public TaskSpecification getNewTaskSpec(){
		return newTaskView.getNewTaskSpec();
	}
	
	public List<String> getNewTaskParamValues(){
		return newTaskView.getNewTaskParamValues();
	}
	
	public String getNewTaskOwner(){
		return newTaskView.getNewTaskOwner();
	}

	public void createNewSubtask() {
		newTaskTypeView.addNewSubtask();
	}
	*/
}
