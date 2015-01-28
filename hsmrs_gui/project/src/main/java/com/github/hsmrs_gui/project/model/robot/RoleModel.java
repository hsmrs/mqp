package src.main.java.com.github.hsmrs_gui.project.model.robot;

import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;

public class RoleModel {

	private String name;
	private List<TaskSpecification> roleTasks;
	
	public RoleModel(String name) {
		this.name = name;
	}

	public RoleModel(String name, List<TaskSpecification> tasks) {
		this(name);
		roleTasks = tasks;
	}
	
	public String getName(){
		return name;
	}
	
	public List<TaskSpecification> getTasks(){
		return roleTasks;
	}
	
	public void setName(String name){
		this.name = name;
	}
	
	public void setTasks(List<TaskSpecification> tasks){
		roleTasks = tasks;
	}
	
}
