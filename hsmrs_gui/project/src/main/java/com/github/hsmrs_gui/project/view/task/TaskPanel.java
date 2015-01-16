package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
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
	
	private TaskPanel(){
		setTaskListModel(TaskListModel.getInstance());
		this.setBorder(BorderFactory.createMatteBorder(1, 1, 3, 3, Color.black));
		this.setLayout(new MigLayout("insets 0, top", "[]", "[]"));
	}
	
	public static TaskPanel getInstance(){
		if (instance == null){
			instance = new TaskPanel();
		}
		
		return instance;
	}
	
	public void setView(int viewCode){
		removeAll();
		validate();
		
		switch (viewCode){
		case TASK_LIST_VIEW:
			add(taskListView, "push, grow, top");
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
		
		validate();
		repaint();
	}
	
	public void switchView(int viewCode) {
		removeAll();
		validate();
		
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
		validate();
		
		repaint();
	}
	
	public void setTaskListModel(TaskListModel tlm){
		taskListView = new TaskListPanel(tlm);
		removeAll();
		add(taskListView, "push, grow, top");
	}
	
	public TaskSpecification getNewTaskSpec(){
		return newTaskView.getNewTaskSpec();
	}
	
	public List<String> getNewTaskParamValues(){
		return newTaskView.getNewTaskParamValues();
	}

	public void createNewSubtask() {
		newTaskTypeView.addNewSubtask();
	}
}
