package src.main.java.com.github.hsmrs_gui.project.model;

import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;

public class RoleModel {

	private String name;
	private List<TaskModel> roleTasks;
	
	public RoleModel(String name) {
		this.name = name;
	}

	public RoleModel(String name, List<TaskModel> tasks) {
		this(name);
		roleTasks = tasks;
	}
	
	public String getName(){
		return name;
	}
	
	public List<TaskModel> getTasks(){
		return roleTasks;
	}
	
	public void setName(String name){
		this.name = name;
	}
	
	public void setTasks(List<TaskModel> tasks){
		roleTasks = tasks;
	}
	
}
