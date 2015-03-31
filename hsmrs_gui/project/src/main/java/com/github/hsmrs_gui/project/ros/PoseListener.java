package src.main.java.com.github.hsmrs_gui.project.ros;

import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
import org.apache.commons.logging.Log;

import com.github.hsmrs_gui.project.GuiNode;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;
import src.main.java.com.github.hsmrs_gui.project.controller.InteractiveMapController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;

public class PoseListener implements MessageListener<geometry_msgs.PoseStamped> {

	private Log log;
	private ConnectedNode connectedNode;
	private RobotModel robotModel;

	/**
	 * Constructor for the PoseListener class.
	 * 
	 * @param robot The model of the robot whose pose is being listened to.
	 * 		   topicName The name of the PoseStamped topic to listen to.
	 */
	public PoseListener(RobotModel robot, String topicName) {
		// Retrieves the node's log and the console in order to
		// record and display status messages
		log = GuiNode.getLog();
		connectedNode = GuiNode.getConnectedNode();
		robotModel = robot;

		// Create the subscriber
		Subscriber<geometry_msgs.PoseStamped> subscriber = connectedNode.newSubscriber(
				topicName, geometry_msgs.PoseStamped._TYPE);
		subscriber.addMessageListener(this);
	}

	/**
	 * This method is called whenever a new PoseStamped message is received
	 * over ROS. It decodes the message and performs the necessary actions.
	 * 
	 * @param message
	 *            The ROS message sent over the topic.
	 */
	@Override
	public void onNewMessage(geometry_msgs.PoseStamped message) {
		geometry_msgs.Pose pose = message.getPose();
		geometry_msgs.Point position = pose.getPosition();
		int x = (int)Math.round(position.getX());
		int y = (int)Math.round(position.getY());
		
		InteractiveMapController.getInstance()
		.updateRobotLocation(robotModel.getName(), 
				new Pair<Integer, Integer>(x, y));
	}
}
