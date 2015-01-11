package src.main.java.com.github.hsmrs_gui.project.ros;

import src.main.java.com.github.hsmrs_gui.project.model.task.Task;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class NewTaskPublisher {

	private Publisher<std_msgs.String> publisher;
	private static NewTaskPublisher instance;
	
	public NewTaskPublisher(ConnectedNode connectedNode){
		publisher =
		        connectedNode.newPublisher("hsmrs/new_task", std_msgs.String._TYPE);
		instance = this;
	}
	
	public static NewTaskPublisher getInstance(){
		return instance;
	}
	
	public void publishNewTask(Task newTask){
		std_msgs.String str = publisher.newMessage();
        str.setData(newTask.toString());
        publisher.publish(str);
	}
}
