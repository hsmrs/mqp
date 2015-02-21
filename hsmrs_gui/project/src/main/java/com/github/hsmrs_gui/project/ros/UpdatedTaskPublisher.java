package src.main.java.com.github.hsmrs_gui.project.ros;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

//import org.ros.rosjava_messages.hsmrs_framework;

public class UpdatedTaskPublisher {

	private Publisher<hsmrs_framework.TaskMsg> publisher;
	private static UpdatedTaskPublisher instance;
	
	public UpdatedTaskPublisher(ConnectedNode connectedNode){
		publisher =
		        connectedNode.newPublisher("hsmrs/updated_task", hsmrs_framework.TaskMsg._TYPE);
		instance = this;
	}
	
	public static UpdatedTaskPublisher getInstance(){
		return instance;
	}
	
	public void publishNewTask(TaskModel newTask){
		hsmrs_framework.TaskMsg taskMsg = publisher.newMessage();
        //str.setData(newTask.toString());
        publisher.publish(taskMsg );
	}
}
