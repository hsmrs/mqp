package src.main.java.com.github.hsmrs_gui.project.model.task;

import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractListModel;

public class TaskSpecificationListModel extends AbstractListModel{
	private static TaskSpecificationListModel instance;
	private List<TaskSpecification> taskSpecList;
	
	/**
	 * The constructor for the TaskSpecificationListModel.
	 */
	private TaskSpecificationListModel(){
		taskSpecList = new ArrayList<TaskSpecification>();
	}
	
	/**
	 * Gets the instance of the only existing TaskSpecificationListModel.
	 * @return The unique instance of the TaskSpecificationListModel.
	 */
	public static TaskSpecificationListModel getInstance(){
		if (instance == null){
			instance = new TaskSpecificationListModel();
		}
		
		return instance;
	}
	
	/**
	 * Adds a TaskSpecification to the list of TaskSpecifications for this system.
	 * @param taskSpec The TaskSpecification to be added.
	 */
	public void addTaskSpecification(TaskSpecification taskSpec){
		taskSpecList.add(taskSpec);
		fireIntervalAdded(this, 0, 0);
	}
	
	/**
	 * Get the list of names of the TaskSpecifications in the list.
	 * @return The list of names of TaskSpecifications.
	 */
	public List<String> getSpecNames(){
		if (taskSpecList.size() == 0){
			ArrayList<String> defaultName = new ArrayList<String>();
			defaultName.add("None");
			return defaultName;
		}
		
		ArrayList<String> returnList = new ArrayList<String>();
		for (TaskSpecification item : taskSpecList){
			returnList.add(item.getName());
		}
		return returnList;
	}

	/**
	 * Gets the TaskSpecification at the given index.
	 * @return The TaskSpecification at the given index. 
	 */
	public TaskSpecification getElementAt(int index) {
		if (index >= taskSpecList.size()){
			ArrayList<String> temp = new ArrayList<String>();
			temp.add(":");
			return new TaskSpecification("None", temp);
		}
		return taskSpecList.get(index);
	}

	/**
	 * Gets the number of TaskSpecifications in the list.
	 * @return The number of TaskSpecifications in the list.
	 */
	public int getSize() {
		return taskSpecList.size();
	}

	/**
	 * Gets a TaskSpecification by its name.
	 * @param taskName The name of the desired TaskSpecification.
	 * @return The TaskSpecification with the desired name.
	 */
	public TaskSpecification getSpecByName(String taskName) {
		for (TaskSpecification ts : taskSpecList){
			if (ts.getName().equals(taskName)){
				return ts;
			}
		}
		return null;
	}
}
