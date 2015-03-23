package src.main.java.com.github.hsmrs_gui.project.ros;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class NewTaskPublisher {

	private Publisher<hsmrs_framework.TaskMsg> publisher;
	private static NewTaskPublisher instance;
	
	/**
	 * Constructor for the NewTaskPublisher class.
	 * @param connectedNode The ConnectedNode for this ROS node.
	 */
	public NewTaskPublisher(ConnectedNode connectedNode){
		publisher =
		        connectedNode.newPublisher("hsmrs/new_task", hsmrs_framework.TaskMsg._TYPE);
		instance = this;
	}
	
	/**
	 * Gets the instance of the NewTaskPublisher.
	 * @return The instance of the NewTaskPublisher.
	 */
	public static NewTaskPublisher getInstance(){
		return instance;
	}
	
	/**
	 * Accepts a TaskModel and publishes a ROS Task message on the new task topic.
	 * @param newTask The new task to be published on ROS.
	 */
	public void publishNewTask(TaskModel newTask){
		hsmrs_framework.TaskMsg msg = publisher.newMessage();
        msg = newTask.toTaskMessage();
        publisher.publish(msg);
	}
}
