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
	private List<Task> taskList;
	private int idCount = 0;
	
	private TaskListModel()
	{
		taskList = new ArrayList<Task>();
		
	}
	
	static public synchronized TaskListModel getInstance() {
		if (instance == null)
			instance = new TaskListModel();
		return instance;
	}
	
	public void addTask(Task newTask){
		idCount++;
		newTask.setID(idCount);
		taskList.add(newTask);
		this.fireIntervalAdded(this, 0, 0);
	}
	
	public Task removeTask(Task targetTask){
		taskList.remove(targetTask);
		this.fireIntervalRemoved(this, 0, 0);
		return targetTask;
	}
	
	public Task removeTask(int index){
		Task targetTask = getElementAt(index);
		taskList.remove(index);
		this.fireIntervalRemoved(this, 0, 0);
		return targetTask;
	}
	
	public Task getElementAt(int index) {
		return taskList.get(index);
	}

	public int getSize() {
		return taskList.size();
	}
}
