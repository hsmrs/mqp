/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	TaskListModel.java												***
***		This class models a list of tasks. It is an implementation	***
***		of the AbstractListModel class.								***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model.task;

import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractListModel;

public class TaskListModel extends AbstractListModel{
	private static TaskListModel instance;
	private List<TaskModel> taskList;
	private int idCount = 0;
	
	private TaskListModel()
	{
		taskList = new ArrayList<TaskModel>();
		
	}
	
	static public synchronized TaskListModel getInstance() {
		if (instance == null)
			instance = new TaskListModel();
		return instance;
	}
	
	public void addTask(TaskModel newTask){
		idCount++;
		newTask.setID(idCount);
		taskList.add(newTask);
		this.fireIntervalAdded(this, 0, 0);
	}
	
	public TaskModel removeTask(TaskModel targetTask){
		taskList.remove(targetTask);
		this.fireIntervalRemoved(this, 0, 0);
		return targetTask;
	}
	
	public TaskModel removeTask(int index){
		TaskModel targetTask = getElementAt(index);
		taskList.remove(index);
		this.fireIntervalRemoved(this, 0, 0);
		return targetTask;
	}
	
	public TaskModel getTaskByID(int taskID){
		for (TaskModel task : taskList){
			if (task.getID() == taskID){
				return task;
			}
		}
		return null;
	}
	
	public TaskModel getElementAt(int index) {
		return taskList.get(index);
	}

	public int getSize() {
		return taskList.size();
	}
}
