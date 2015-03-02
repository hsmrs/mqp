package src.main.java.com.github.hsmrs_gui.project.model.robot;

import java.util.ArrayList;
import java.util.List;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;

public class RoleModel {

	private String type;
	private List<TaskSpecification> roleTasks;
	
	public RoleModel(String type) {
		this.type = type;
	}

	public RoleModel(String type, List<TaskSpecification> tasks) {
		this(type);
		roleTasks = tasks;
	}
	
	public String getType(){
		return type;
	}
	
	public List<TaskSpecification> getTasks(){
		return roleTasks;
	}
	
	public void setType(String type){
		this.type = type;
	}
	
	public void setTasks(List<TaskSpecification> tasks){
		roleTasks = tasks;
	}
	
	public hsmrs_framework.RoleMsg toMessage(){
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
		MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
		hsmrs_framework.RoleMsg msg = messageFactory.newFromType(hsmrs_framework.RoleMsg._TYPE);
		
		msg.setType(type);
		
		List<String> taskTypes = new ArrayList<String>();
		for (TaskSpecification ts : roleTasks){
			taskTypes.add(ts.getType());
		}
		msg.setTaskTypes(taskTypes);
		
		return msg;
	}
	
}
