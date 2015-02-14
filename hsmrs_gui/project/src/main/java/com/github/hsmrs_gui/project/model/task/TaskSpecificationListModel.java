package src.main.java.com.github.hsmrs_gui.project.model.task;

import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractListModel;

public class TaskSpecificationListModel extends AbstractListModel{
	private static TaskSpecificationListModel instance;
	private List<TaskSpecification> taskSpecList;
	
	private TaskSpecificationListModel(){
		taskSpecList = new ArrayList<TaskSpecification>();
	}
	
	public static TaskSpecificationListModel getInstance(){
		if (instance == null){
			instance = new TaskSpecificationListModel();
		}
		
		return instance;
	}
	
	public void addTaskSpecification(TaskSpecification taskSpec){
		taskSpecList.add(taskSpec);
		fireIntervalAdded(this, 0, 0);
	}
	
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

	public TaskSpecification getElementAt(int index) {
		if (index >= taskSpecList.size()){
			ArrayList<String> temp = new ArrayList<String>();
			temp.add(":");
			return new TaskSpecification("None", temp);
		}
		return taskSpecList.get(index);
	}

	public int getSize() {
		return taskSpecList.size();
	}

	public TaskSpecification getSpecByName(String taskType) {
		for (TaskSpecification ts : taskSpecList){
			if (ts.getName().equals(taskType)){
				return ts;
			}
		}
		return null;
	}
}
