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
	
	/**
	 * The constructor for the RobotPanel class, which wraps all of the subpanels dealing with displaying Robot and Role information.
	 */
	private RobotPanel(){
		this.setBorder(BorderFactory.createMatteBorder(1, 1, 3, 3, Color.black));
		//this.setLayout(new MigLayout("insets 0, top, fill, debug", "[]", "[]"));
		this.setLayout(new GridLayout());
		robotListView = RobotListView.getInstance();
		RobotController.getInstance().setRobotView(this);
		add(robotListView);
	}
	
	/**
	 * Gets the instance of the only existing RobotPanel.
	 * @return The instance of the only existing RobotPanel.
	 */
	public static RobotPanel getInstance(){
		if (instance == null){
			instance = new RobotPanel();
		}
		
		return instance;
	}
	
	/**
	 * Sets which subpanel is being displayed. THis creates a new instance of the subpanel.
	 * @param viewCode The integer code for the subview, given by the consts defined in this class.
	 */
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
		validate();
		repaint();
	}
	
	/**
	 * Update the robot list subpanel. 
	 */
	public void updateRobotList(){
		robotListView.update();
	}
	
	/**
	 * Sets which subpanel is being viewed. This does not create new instances of the subpanels.
	 * @param viewCode The integer code for the subview, given by the consts defined in this class. 
	 */
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
	
	/**
	 * Sets the underlying list model for this view.
	 * @param rlm
	 */
	public void setListModel(ListModel rlm){
		robotListView.setListModel(rlm);
	}

	/**
	 * Gets the role that is selected in the RoleListView.
	 * @return The role that is selected in the RoleListView.
	 */
	public RoleModel getSelectedRole() {
		return roleListPanel.getSelectedRole();
	}

	/**
	 * Gets the robot that was targeted for a new role.
	 * @return The robot being given a new role.
	 */
	public RobotModel getTargetRobot() {
		return targetRobot;
	}

	/**
	 * Set the robot that is being targeted for a new role.
	 * @param robot The robot being given a new role.
	 */
	public void setTargetRobot(RobotModel robot) {
		targetRobot = robot;		
	}

	/**
	 * Gets the new role being given to the target robot.
	 * @return The new role being given to the target robot.
	 */
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
