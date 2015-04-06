package src.main.java.com.github.hsmrs_gui.project.ros;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.github.hsmrs_gui.project.GuiNode;

public class TeleOpPublisher {
	
	public static final String UP = "up";
	public static final String DOWN = "down";
	public static final String LEFT = "left";
	public static final String RIGHT = "right";
	public static final String STOP = "stop";
	
	private Publisher<geometry_msgs.Twist> publisher;
	private ConnectedNode connectedNode;
	
	/**
	 * The constructor of the TeleOpPublisher class.
	 * @param topicName The topic to publish tele-op messages to.
	 */
	public TeleOpPublisher(String topicName){
		connectedNode = GuiNode.getConnectedNode();
		publisher =
		        connectedNode.newPublisher(topicName, geometry_msgs.Twist._TYPE);
	}
	
	/**
	 * Publishes a twist message which corresponds to the given direction.
	 * @param direction
	 */
	public void publishMessage(String direction){
		geometry_msgs.Twist msg = publisher.newMessage();
		if (direction.equals(UP)){
			msg.getLinear().setX(1);
		}
		else if (direction.equals(DOWN)){
			msg.getLinear().setX(-1);
		}
		else if (direction.equals(LEFT)){
			msg.getLinear().setY(1);
		}
		else if (direction.equals(RIGHT)){
			msg.getLinear().setY(-1);
		}
		//No else is needed for STOP, just publish empty twist.
		
        publisher.publish(msg);
	}

}
