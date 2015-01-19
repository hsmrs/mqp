package src.main.java.com.github.hsmrs_gui.project.controller;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
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
	
	public void addOwnerToTask(TaskModel task, String ownerName){
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(ownerName);
		task.addOwner(robot);
		TaskPanel.getInstance().updateTaskList();
	}
	
	public void removeOwnerFromTask(TaskModel task, String ownerName){
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(ownerName);
		task.removeOwner(robot);
		TaskPanel.getInstance().updateTaskList();
	}

	public void actionPerformed(ActionEvent e) {
		String cmd = e.getActionCommand();
		TaskPanel tp = TaskPanel.getInstance();
		
		if (cmd.equals("New task")){
			tp.setView(TaskPanel.NEW_TASK_VIEW);
		}
		else if (cmd.equals("Remove task")){
			List<TaskModel> targets = tp.getSelectedTask();
			TaskListModel tlm = TaskListModel.getInstance();
			for (TaskModel task : targets){
				tlm.removeTask(task);
			}
		}
		else if (cmd.equals("New task type")){
			tp.setView(TaskPanel.NEW_TASK_TYPE_VIEW);
		}
		else if (cmd.equals("Create task")){
			TaskSpecification spec = tp.getNewTaskSpec();
			String taskName = spec.getName();
			ArrayList<TaskParam<?>> taskParams = new ArrayList<TaskParam<?>>();
			List<String> paramValues = tp.getNewTaskParamValues();
			String owner = tp.getNewTaskOwner();
			
			for (int i = 0; i < paramValues.size(); i++){
				String paramValueStr = paramValues.get(i);
				String paramTypePair = spec.getParameterTypePairs().get(i);
				String expectedType = paramTypePair.split(":")[1];
				
				if (expectedType.equals("Integer")){
					TaskParam<Integer> tpa = new TaskParam<Integer>(
							paramTypePair.split(":")[0],
							Integer.parseInt(paramValueStr));
					taskParams.add(tpa);
				}
				else if (expectedType.equals("Double")){
					TaskParam<Double> tpa = new TaskParam<Double>(
							paramTypePair.split(":")[0],
							Double.parseDouble(paramValueStr));
					taskParams.add(tpa);
				}
				else if (expectedType.equals("String")){
					TaskParam<String> tpa = new TaskParam<String>(
							paramTypePair.split(":")[0],
							paramValueStr);
					taskParams.add(tpa);
				}
				else {
					System.out.println("Did not recognize type: " + expectedType);
				}
			}
			
			TaskModel newTask = new TaskModel(taskName, taskParams);
			if (!owner.equals("None")){
				newTask.addOwner(RobotListModel.getInstance()
						.getRobotModelByName(owner));
			}
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
