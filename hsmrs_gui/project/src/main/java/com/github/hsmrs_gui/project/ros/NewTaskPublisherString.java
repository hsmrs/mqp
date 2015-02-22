package src.main.java.com.github.hsmrs_gui.project.ros;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class NewTaskPublisherString {

	private Publisher<std_msgs.String> publisher;
	private static NewTaskPublisherString instance;
	
	public NewTaskPublisherString(ConnectedNode connectedNode){
		publisher =
		        connectedNode.newPublisher("hsmrs/new_task", std_msgs.String._TYPE);
		instance = this;
	}
	
	public static NewTaskPublisherString getInstance(){
		return instance;
	}
	
	public void publishNewTask(TaskModel newTask){
		std_msgs.String str = publisher.newMessage();
        str.setData(newTask.toString());
        publisher.publish(str);
	}
}
