package src.main.java.com.github.hsmrs_gui.project.ros;

import com.github.hsmrs_gui.project.GuiNode;

import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
import org.apache.commons.logging.Log;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;
import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;

public class HelpListener implements MessageListener<std_msgs.String>{

	private Log log;
	private ConnectedNode connectedNode;
	private RobotModel robot;
	
	/**
	 * The constructor for the StatusListener class.
	 * @param topicName The name of the ROS topic
	 */
	public HelpListener(RobotModel robot, String topicName) {
		log = GuiNode.getLog();
		connectedNode = GuiNode.getConnectedNode();
		this.robot = robot;

		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				topicName, std_msgs.String._TYPE);
		subscriber.addMessageListener(this);
	}

	/**
	 * This method is called whenever a message is received through ROS.
	 * @param message The String message received through the topic.
	 */
	@Override
	public void onNewMessage(std_msgs.String message) {
		boolean needsHelp = Boolean.valueOf(message.getData());
		
		RobotController.getInstance().setRobotNeedsHelp(robot.getName(), needsHelp);
	}

}
