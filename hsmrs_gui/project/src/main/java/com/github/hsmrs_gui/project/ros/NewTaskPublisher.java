package src.main.java.com.github.hsmrs_gui.project.ros;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class NewTaskPublisher {

	private Publisher<hsmrs_framework.TaskMsg> publisher;
	private static NewTaskPublisher instance;
	
	public NewTaskPublisher(ConnectedNode connectedNode){
		publisher =
		        connectedNode.newPublisher("hsmrs/new_task", hsmrs_framework.TaskMsg._TYPE);
		instance = this;
	}
	
	public static NewTaskPublisher getInstance(){
		return instance;
	}
	
	public void publishNewTask(TaskModel newTask){
		hsmrs_framework.TaskMsg msg = publisher.newMessage();
        msg = newTask.toTaskMessage();
        publisher.publish(msg);
	}
}
