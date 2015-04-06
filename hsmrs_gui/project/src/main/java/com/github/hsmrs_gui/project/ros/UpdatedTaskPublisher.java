package src.main.java.com.github.hsmrs_gui.project.ros;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.github.hsmrs_gui.project.GuiNode;

public class UpdatedTaskPublisher {

	private Publisher<hsmrs_framework.TaskMsg> publisher;
	private static UpdatedTaskPublisher instance;
	
	/**
	 * The constructor for the UpdatedTaskPublisher.
	 */
	private UpdatedTaskPublisher(){
		ConnectedNode connectedNode = GuiNode.getConnectedNode();
		publisher =
		        connectedNode.newPublisher("hsmrs/updated_task", hsmrs_framework.TaskMsg._TYPE);
	}
	
	/**
	 * Gets the only existing instance of the UpdatedTaskPublisher.
	 * @return The only existing instance of the UpdatedTaskPublisher.
	 */
	public static UpdatedTaskPublisher getInstance(){
		if (instance == null){
			instance = new UpdatedTaskPublisher();
		}
		return instance;
	}
	
	/**
	 * Publishes the given task as a ROS Task message.
	 * @param task The updated task to be published.
	 */
	public void publishUpdatedTask(TaskModel task){
		hsmrs_framework.TaskMsg taskMsg = task.toTaskMessage();
        publisher.publish(taskMsg );
	}
	
	/**
	 * Publishes the given task with a status of deleted.
	 * @param deletedTask The deleted task.
	 */
	public void publishDeletedTask(TaskModel deletedTask){
		hsmrs_framework.TaskMsg taskMsg = deletedTask.toTaskMessage();
		taskMsg.setStatus("deleted");
		publisher.publish(taskMsg );
	}
}
