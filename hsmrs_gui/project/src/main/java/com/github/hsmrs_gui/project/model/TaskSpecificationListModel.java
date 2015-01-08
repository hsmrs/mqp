package src.main.java.com.github.hsmrs_gui.project.model;

import java.util.List;

import javax.swing.AbstractListModel;

public class TaskSpecificationListModel extends AbstractListModel{
	private static TaskSpecificationListModel instance;
	private List<TaskSpecification> taskSpecList;
	
	private TaskSpecificationListModel(){}
	
	public static TaskSpecificationListModel getInstance(){
		if (instance == null){
			instance = new TaskSpecificationListModel();
		}
		
		return instance;
	}
	
	public void addTaskSpecification(TaskSpecification taskSpec){
		taskSpecList.add(taskSpec);
	}

	public TaskSpecification getElementAt(int index) {
		return taskSpecList.get(index);
	}

	public int getSize() {
		return taskSpecList.size();
	}
}
