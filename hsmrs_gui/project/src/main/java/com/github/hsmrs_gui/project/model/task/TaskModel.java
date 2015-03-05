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
	
	public TaskModel(){
		type = "Idle";
		paramList = new ArrayList<TaskParam<?>>();
		owners = new ArrayList<RobotModel>();
		status = "---";
		subTasks = new ArrayList<TaskModel>();
	}
	
	public TaskModel(String name, List<TaskParam<?>> paramList){
		this.type = name;
		this.paramList = paramList;
		owners = new ArrayList<RobotModel>();
		status = "Not started";
		subTasks = new ArrayList<TaskModel>();
	}
	
	public TaskModel(String name, List<TaskParam<?>> paramList,
			List<RobotModel> owners, String status){
		this(name, paramList);
		
		this.owners = owners;
		this.status = status;
	}
	
	public synchronized int getID(){ 
		return id;
	}
	
	public synchronized void setID(int newID){
		this.id = newID;
	}
	
	public synchronized String getType() {
		return type;
	}

	public synchronized void setType(String type) {
		this.type = type;
	}

	public synchronized List<RobotModel> getOwners() {
		return owners;
	}

	public synchronized void addOwner(RobotModel owner) {
		owners.add(owner);
	}
	
	public synchronized void removeOwner(RobotModel owner){
		owners.remove(owner);
	}

	public synchronized String getStatus() {
		return status;
	}

	public synchronized void setStatus(String status) {
		this.status = status;
	}
	
	public synchronized void setPriority(double priority) {
		this.priority = priority;
	}
	
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
