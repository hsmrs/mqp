package src.main.java.com.github.hsmrs_gui.project.model.robot;

import java.util.ArrayList;
import java.util.List;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;

public class RoleModel {

	private String type;
	private List<TaskSpecification> roleTasks;
	
	/**
	 * Constructor for the RoleModel class.
	 * @param type The type of role
	 */
	public RoleModel(String type) {
		this.type = type;
	}

	/**
	 * The constructor for the RoleModel class by type and list of tasks included within the role.
	 * @param type The type of role
	 * @param tasks The list of tasks that a robot can accept with this role.
	 */
	public RoleModel(String type, List<TaskSpecification> tasks) {
		this(type);
		roleTasks = tasks;
	}
	
	/**
	 * Gets the type of this role
	 * @return The type of this role.
	 */
	public String getType(){
		return type;
	}
	
	/**
	 * Gets the list of tasks admissible by this role.
	 * @return The list of tasks admissible by this role.
	 */
	public List<TaskSpecification> getTasks(){
		return roleTasks;
	}
	
	/**
	 * Set the type of this role.
	 * @param type The new type of this role.
	 */
	public void setType(String type){
		this.type = type;
	}
	
	/**
	 * Set the list of tasks admissible by this role.
	 * @param tasks The new list of admissible tasks.
	 */
	public void setTasks(List<TaskSpecification> tasks){
		roleTasks = tasks;
	}
	
	/**
	 * Convert this role to a ROS message
	 * @return A ROS message version of this role.
	 */
	public hsmrs_framework.RoleMsg toMessage(){
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
		MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
		hsmrs_framework.RoleMsg msg = messageFactory.newFromType(hsmrs_framework.RoleMsg._TYPE);
		
		msg.setType(type);
		
		List<String> taskTypes = new ArrayList<String>();
		for (TaskSpecification ts : roleTasks){
			taskTypes.add(ts.getName() + "Task");
		}
		msg.setTaskTypes(taskTypes);
		
		return msg;
	}
	
}
