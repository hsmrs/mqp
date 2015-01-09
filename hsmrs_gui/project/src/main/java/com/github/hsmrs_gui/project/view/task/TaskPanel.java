package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;

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
		this.setLayout(new MigLayout("insets 0", "[]", "[]"));
	}
	
	public static TaskPanel getInstance(){
		if (instance == null){
			instance = new TaskPanel();
		}
		
		return instance;
	}
	
	public void setView(int viewCode){
		System.out.println("Set Task view");
		removeAll();
		validate();
		
		switch (viewCode){
		case TASK_LIST_VIEW:
			add(taskListView);
			break;
		case NEW_TASK_VIEW:
			add(new NewTaskPanel());
			break;
		case NEW_TASK_TYPE_VIEW:
			add(new NewTaskTypePanel());
			break;
		}
		validate();
		
		System.out.println("Repaint");
		repaint();
	}
	
	public void setTaskListModel(TaskListModel tlm){
		taskListView = new TaskListPanel(tlm);
		removeAll();
		add(taskListView);
	}
}
