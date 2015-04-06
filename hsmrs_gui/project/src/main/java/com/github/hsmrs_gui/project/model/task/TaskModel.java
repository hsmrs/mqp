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

import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;

public class TaskModel {

	private int id;
	private String type;
	private List<TaskParam<?>> paramList;
	private List<TaskModel> subTasks;
	private List<RobotModel> owners;
	private String status;
	private double priority;
	
	/**
	 * The constructor for the TaskModel class.
	 */
	public TaskModel(){
		type = "Idle";
		paramList = new ArrayList<TaskParam<?>>();
		owners = new ArrayList<RobotModel>();
		status = "---";
		subTasks = new ArrayList<TaskModel>();
	}
	
	/**
	 * The constructor of the TaskModel class by name and the list of parameters.
	 * @param name The name of this task.
	 * @param paramList The list of parameters for this task.
	 */
	public TaskModel(String name, List<TaskParam<?>> paramList){
		this.type = name;
		this.paramList = paramList;
		owners = new ArrayList<RobotModel>();
		status = "Not started";
		subTasks = new ArrayList<TaskModel>();
	}
	
	/**
	 * The constructor for the TaskModel class by name, parameter list, owner list, and status.
	 * @param name The name of this task.
	 * @param paramList The list of the parameters for this task.
	 * @param owners The list of owners of this task.
	 * @param status The status of this task.
	 */
	public TaskModel(String name, List<TaskParam<?>> paramList,
			List<RobotModel> owners, String status){
		this(name, paramList);
		
		this.owners = owners;
		this.status = status;
	}
	
	/**
	 * Gets the ID of this task.
	 * @return The ID of this task.
	 */
	public synchronized int getID(){ 
		return id;
	}
	
	/**
	 * Sets the ID of this task.
	 * @param newID The new ID of this task.
	 */
	public synchronized void setID(int newID){
		this.id = newID;
	}
	
	/**
	 * Gets the type of the task.
	 * @return The type of this task.
	 */
	public synchronized String getType() {
		return type;
	}

	/**
	 * Sets the type of this task.
	 * @param type The new type of this task.
	 */
	public synchronized void setType(String type) {
		this.type = type;
	}

	/**
	 * Gets the list of owners of this task.
	 * @return The list of owners.
	 */
	public synchronized List<RobotModel> getOwners() {
		return owners;
	}

	/**
	 * Adds an owner to this TaskModel.
	 * @param owner The new owner of this task.
	 */
	public synchronized void addOwner(RobotModel owner) {
		owners.add(owner);
	}
	
	/**
	 * Remove an owner from the TaskModel.
	 * @param owner The owner to be removed from the task.
	 */
	public synchronized void removeOwner(RobotModel owner){
		owners.remove(owner);
	}

	/**
	 * Gets the status of this task.
	 * @return The status of this task.
	 */
	public synchronized String getStatus() {
		return status;
	}

	/**
	 * Sets the status of this task.
	 * @param status The new status of this task.
	 */
	public synchronized void setStatus(String status) {
		this.status = status;
	}
	
	/**
	 * Sets the priority of this task.
	 * @param priority The new priority of this task.
	 */
	public synchronized void setPriority(double priority) {
		this.priority = priority;
	}
	
	/**
	 * Converts this TaskModel to a ROS message.
	 * @return A ROS message representing this TaskModel.
	 */
	public synchronized hsmrs_framework.TaskMsg toTaskMessage(){
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
		MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
		hsmrs_framework.TaskMsg msg = messageFactory.newFromType(hsmrs_framework.TaskMsg._TYPE); 
		
		msg.setId(id);
		msg.setParentId(hsmrs_framework.TaskMsg.NO_PARENT);
		msg.setType(type+"Task");
		
		List<String> paramValues = new ArrayList<String>();
		for (TaskParam<?> taskParam : paramList){
			paramValues.add(taskParam.getValue().toString());
		}
		msg.setParamValues(paramValues);
		
		msg.setPriority((float)priority);
		
		List<String> ownerNames = new ArrayList<String>();
		for (RobotModel robot : owners){
			ownerNames.add(robot.getName());
		}
		msg.setOwners(ownerNames);
		
		return msg;
	}

	/**
	 * Converts this TaskModel to a String.
	 * @return The String representation of this String.
	 */
	public String toString(){
		/*
		 * ID;Name;[ParamName:ParamType=ParamValue, ...];[SubTask1, SubTask2, ...];[Owner1, Owner2, ...]
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
				type + ";" +
				paramString +
				subTaskString;			
	}
}
