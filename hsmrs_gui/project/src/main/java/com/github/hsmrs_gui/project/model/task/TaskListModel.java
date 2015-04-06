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
	
	/**
	 * The constructor for the TaskListModel class.
	 */
	private TaskListModel()
	{
		taskList = new ArrayList<TaskModel>();
		
	}
	
	/**
	 * Returns the instance of the only existing TaskListModel.
	 * @return The instance of the TaskListModel.
	 */
	static public synchronized TaskListModel getInstance() {
		if (instance == null)
			instance = new TaskListModel();
		return instance;
	}
	
	/**
	 * Adds a task to the list of tasks in the system.
	 * @param newTask The task to add to the system.
	 */
	public synchronized void addTask(TaskModel newTask){
		idCount++;
		newTask.setID(idCount);
		taskList.add(newTask);
		this.fireIntervalAdded(this, 0, 0);
	}
	
	/**
	 * Removes a task from the system.
	 * @param targetTask The task to be removed.
	 * @return The removed task.
	 */
	public synchronized TaskModel removeTask(TaskModel targetTask){
		taskList.remove(targetTask);
		this.fireIntervalRemoved(this, 0, 0);
		return targetTask;
	}
	
	/**
	 * Removes a task by its index.
	 * @param index The index of the task to remove.
	 * @return The removed task.
	 */
	public synchronized TaskModel removeTask(int index){
		TaskModel targetTask = getElementAt(index);
		taskList.remove(index);
		this.fireIntervalRemoved(this, 0, 0);
		return targetTask;
	}
	
	/**
	 * Gets the task by its ID.
	 * @param taskID The ID of the task
	 * @return The task with the given ID.
	 */
	public synchronized TaskModel getTaskByID(int taskID){
		for (TaskModel task : taskList){
			if (task.getID() == taskID){
				return task;
			}
		}
		return null;
	}
	
	/**
	 * Gets the TaskModel at the given index.
	 * @param index The index of the desired Task.
	 * @return The TaskModel at the given index.
	 */
	public synchronized TaskModel getElementAt(int index) {
		return taskList.get(index);
	}

	/**
	 * Returns the number of tasks in the system.
	 * @return The number of tasks in the system.
	 */
	public synchronized int getSize() {
		return taskList.size();
	}
}
