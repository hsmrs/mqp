/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	Task.java														***
***		This class models a task. A task has a name, a list of		***
***		owners, a list of subtasks, and a status. 					***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model.task;

import java.util.ArrayList;
import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;

public class TaskModel {

	private int id;
	private String name;
	private List<TaskParam<?>> paramList;
	private List<TaskModel> subTasks;
	private List<RobotModel> owners;
	private String status;
	
	public TaskModel(){
		name = "Idle";
		paramList = new ArrayList<TaskParam<?>>();
		owners = new ArrayList<RobotModel>();
		status = "---";
		subTasks = new ArrayList<TaskModel>();
	}
	
	public TaskModel(String name, List<TaskParam<?>> paramList){
		this.name = name;
		this.paramList = paramList;
		owners = new ArrayList<RobotModel>();
		status = "Not claimed";
		subTasks = new ArrayList<TaskModel>();
	}
	
	public TaskModel(String name, List<TaskParam<?>> paramList,
			List<RobotModel> owners, String status){
		this(name, paramList);
		
		this.owners = owners;
		this.status = status;
	}
	
	public int getID(){
		return id;
	}
	
	public void setID(int newID){
		this.id = newID;
	}
	
	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public List<RobotModel> getOwners() {
		return owners;
	}

	public void addOwner(RobotModel owner) {
		owners.add(owner);
	}
	
	public void removeOwner(RobotModel owner){
		owners.remove(owner);
	}

	public String getStatus() {
		return status;
	}

	public void setStatus(String status) {
		this.status = status;
	}

	public String toString(){
		/*
		 * ID;Name;[ParamName:ParamType=ParamValue, ...];[SubTask1, SubTask2, ...]
		 */
		StringBuilder sb = new StringBuilder();
		sb.append("[");
		for (TaskParam<?> tp : paramList){
			sb.append(tp.getLabel() + ":" + tp.getType() + "=" + 
		tp.getValue().toString() + ",");
		}
		if (paramList.size() > 0) sb.deleteCharAt(sb.length() - 1);
		sb.append("];");
		String paramString = sb.toString();
		
		sb = new StringBuilder();
		sb.append("[");
		for (TaskModel tk : subTasks){
			sb.append(tk.toString() + ",");
		}
		if (subTasks.size() > 0) sb.deleteCharAt(sb.length() - 1);
		sb.append("];");
		
		sb.append("[");
		for (RobotModel owner : owners){
			sb.append(owner.getName() + ",");
		}
		if (owners.size() > 0) sb.deleteCharAt(sb.length() - 1);
		sb.append("]");
		
		String subTaskString = sb.toString();
		
		return String.valueOf(id) + ";" +
				name + ";" +
				paramString +
				subTaskString;			
	}
}
