package src.main.java.com.github.hsmrs_gui.project.controller;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskPanel;

public class TaskController implements ActionListener{
	
	private static TaskController instance;
	
	private TaskController(){}
	
	public static TaskController getInstance(){
		if (instance == null){
			instance = new TaskController();
		}
		
		return instance;
	}

	public void actionPerformed(ActionEvent e) {
		System.out.println("Task action!");
		String cmd = e.getActionCommand();
		
		if (cmd.equals("New task")){
			System.out.println("New Task");
			TaskPanel.getInstance().setView(TaskPanel.NEW_TASK_VIEW);
		}
		else if (cmd.equals("Remove task")){
			
		}
		else if (cmd.equals("New task type")){
			
		}
		else if (cmd.equals("Create task")){
			
		}
		else if (cmd.equals("Cancel new task")){
			TaskPanel.getInstance().setView(TaskPanel.TASK_LIST_VIEW);
		}
	}

}
