package src.main.java.com.github.hsmrs_gui.project.ros;

import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
import org.apache.commons.logging.Log;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;

import com.github.hsmrs_gui.project.GuiNode;

public class RobotRegistrationListener implements
		MessageListener<std_msgs.String> {
	private Log log;
	private ConsoleController consoleController;

	public RobotRegistrationListener(ConnectedNode connectedNode) {
		log = GuiNode.getLog();
		consoleController = ConsoleController.getInstance();

		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				"hsmrs/robot_configuration", std_msgs.String._TYPE);
		subscriber.addMessageListener(this);
	}

	@Override
	public void onNewMessage(std_msgs.String message) {
		String[] messageData = message.getData().split(";");

		log.info("Confirm registration: " + messageData[0]);
		RobotListModel rlm = RobotListModel.getRobotListModel();
		RobotModel newRobot = new RobotModel(messageData[0]);
		newRobot.setImageTopic(messageData[1]);
		newRobot.setImageListener(new ImageListener(messageData[1]));
		
		rlm.addRobot(newRobot);
		log.info("Successful registration: " + messageData[0]);
		long timestamp = System.currentTimeMillis() / 1000;
		consoleController.addLog("System", timestamp, "New agent has been successfully registered: "
						+ messageData[0]);		
	}
}
