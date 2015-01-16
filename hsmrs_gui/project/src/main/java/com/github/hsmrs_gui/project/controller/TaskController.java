package src.main.java.com.github.hsmrs_gui.project.controller;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.model.task.Task;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskParam;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.ros.NewTaskPublisher;
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
		String cmd = e.getActionCommand();
		
		if (cmd.equals("New task")){
			TaskPanel.getInstance().setView(TaskPanel.NEW_TASK_VIEW);
		}
		else if (cmd.equals("Remove task")){
			
		}
		else if (cmd.equals("New task type")){
			TaskPanel.getInstance().setView(TaskPanel.NEW_TASK_TYPE_VIEW);
		}
		else if (cmd.equals("Create task")){
			TaskSpecification spec = TaskPanel.getInstance().getNewTaskSpec();
			String taskName = spec.getName();
			ArrayList<TaskParam<?>> taskParams = new ArrayList<TaskParam<?>>();
			List<String> paramValues = TaskPanel.getInstance().getNewTaskParamValues();
			
			for (int i = 0; i < paramValues.size(); i++){
				String paramValueStr = paramValues.get(i);
				String paramTypePair = spec.getParameterTypePairs().get(i);
				String expectedType = paramTypePair.split(":")[1];
				
				if (expectedType.equals("Integer")){
					TaskParam<Integer> tp = new TaskParam<Integer>(
							paramTypePair.split(":")[0],
							Integer.parseInt(paramValueStr));
					taskParams.add(tp);
				}
				else if (expectedType.equals("Double")){
					TaskParam<Double> tp = new TaskParam<Double>(
							paramTypePair.split(":")[0],
							Double.parseDouble(paramValueStr));
					taskParams.add(tp);
				}
				else if (expectedType.equals("String")){
					TaskParam<String> tp = new TaskParam<String>(
							paramTypePair.split(":")[0],
							paramValueStr);
					taskParams.add(tp);
				}
				else {
					System.out.println("Did not recognize type: " + expectedType);
				}
			}
			
			Task newTask = new Task(taskName, taskParams);
			TaskListModel.getInstance().addTask(newTask);
			NewTaskPublisher.getInstance().publishNewTask(newTask);
			System.out.println("New task created!");
			
			TaskPanel.getInstance().setView(TaskPanel.TASK_LIST_VIEW);
		}
		else if (cmd.equals("Cancel new task")){
			TaskPanel.getInstance().setView(TaskPanel.TASK_LIST_VIEW);
		}
		else if (cmd.equals("New subtask")){
			TaskPanel.getInstance().createNewSubtask();
		}
		else if (cmd.equals("Create task type")){
			
		}
		else if (cmd.equals("Cancel new task type")){
			TaskPanel.getInstance().switchView(TaskPanel.NEW_TASK_VIEW);
		}
	}

}
