package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.GridLayout;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import net.miginfocom.swing.MigLayout;


public class TaskPanel extends JPanel{
	private static TaskPanel instance;
	private TaskListPanel taskListView;
	private NewTaskPanel newTaskView;
	private NewTaskTypePanel newTaskTypeView;
	
	public static final int TASK_LIST_VIEW = 0;
	public static final int NEW_TASK_VIEW = 1;
	public static final int NEW_TASK_TYPE_VIEW = 2;
	
	/**
	 * The constructor for the TaskPanel class.
	 */
	private TaskPanel(){
		setTaskListModel(TaskListModel.getInstance());
		this.setBorder(BorderFactory.createMatteBorder(1, 1, 3, 3, Color.black));
		//this.setLayout(new MigLayout("insets 0, top, fill, debug", "[]", "[]"));
		this.setLayout(new GridLayout());
	}
	
	/**
	 * Gets the only existing instance of the TaskPanel class.
	 * @return The only existing instance of the TaskPanel class.
	 */
	public static TaskPanel getInstance(){
		if (instance == null){
			instance = new TaskPanel();
		}
		
		return instance;
	}
	
	/**
	 * Sets which subpanel is displayed on the TaskPanel view. This creates a new instance of the subpanel.
	 * @param viewCode The code for which subpanel will be displayed. Codes are const variables defined in this class.
	 */
	public void setView(int viewCode){
		removeAll();
		validate();
		
		switch (viewCode){
		case TASK_LIST_VIEW:
			add(taskListView);//, BorderLayout.CENTER);
			//add(taskListView, "push, grow, top");
			break;
		case NEW_TASK_VIEW:
			newTaskView = new NewTaskPanel();
			add(newTaskView, "push, grow, top");
			break;
		case NEW_TASK_TYPE_VIEW:
			newTaskTypeView = new NewTaskTypePanel();
			add(newTaskTypeView, "push, grow, top");
			break;
		}
		
		//add(new JLabel("I HATE THIS"));
		
		validate();
		repaint();
	}
	
	/**
	 * Updates the TaskList subpanel.
	 */
	public void updateTaskList(){
		taskListView.update();
	}
	
	/**
	 * Sets which subpanel is displayed on the TaskPanel view. This does not create a new instance of the subpanel.
	 * @param viewCode The code for which subpanel will be displayed. Codes are const variables defined in this class.
	 */
	public void switchView(int viewCode) {
		removeAll();
		
		switch (viewCode){
		case TASK_LIST_VIEW:
			add(taskListView, "push, grow, top");
			break;
		case NEW_TASK_VIEW:
			add(newTaskView, "push, grow, top");
			break;
		case NEW_TASK_TYPE_VIEW:
			add(newTaskTypeView, "push, grow, top");
			break;
		}
		revalidate();
		
		repaint();
	}
	
	/**
	 * Sets the underlying list model for the TaskListView subpanel.
	 * @param tlm
	 */
	public void setTaskListModel(TaskListModel tlm){
		taskListView = new TaskListPanel(tlm);
		setView(TASK_LIST_VIEW);
		revalidate();
		repaint();
	}
	
	/**
	 * Get the selected task from the TaskListView.
	 * @return The selected task from the TaskListView.
	 */
	public List<TaskModel> getSelectedTask(){
		return taskListView.getSelectedTask();
	}
	
	/**
	 * Get the spec for the new task created in the NewTaskView.
	 * @return The spec for the new task created in the NewTaskView.
	 */
	public TaskSpecification getNewTaskSpec(){
		return newTaskView.getNewTaskSpec();
	}
	
	/**
	 * Gets the parameters for the new task created in the NewTaskView.
	 * @return The list of parameters as strings.
	 */
	public List<String> getNewTaskParamValues(){
		return newTaskView.getNewTaskParamValues();
	}

	/**
	 * Gets the priority of the new task created in the NewTaskView.
	 * @return The new task's priority.
	 */
	public double getNewTaskPriority() {
		return newTaskView.getNewTaskPrioirty();
	}
	
	/**
	 * Gets the owner of the new task created in the NewTaskView.
	 * @return The owner of the new task.
	 */
	public String getNewTaskOwner(){
		return newTaskView.getNewTaskOwner();
	}

	/**
	 * Delegates the creation of a new Subtask to the NewTaskTypeView.
	 */
	public void createNewSubtask() {
		newTaskTypeView.addNewSubtask();
	}
}
