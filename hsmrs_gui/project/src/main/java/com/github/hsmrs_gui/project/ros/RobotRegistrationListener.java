/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	RobotRegistrationListener.java									***
***		This class defines a ROS subscriber which listens for		***
***		String messages which encode registrations of robots with	***
***		the HSMRS system. It handles all of the operations involved	***
***		with adding a robot to the system.							***
***																	***
***		Registration messages are delimited by semi-colons and have	***
***		the following fields: <Name>;<Image Topic>					***
***		Example: "Oryx;oryx/camera/image_rgb"						***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.ros;

import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
import org.apache.commons.logging.Log;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;
import src.main.java.com.github.hsmrs_gui.project.controller.InteractiveMapController;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;
import src.main.java.com.github.hsmrs_gui.project.view.situational_awareness.InteractiveMapViewLayered;

import com.github.hsmrs_gui.project.GuiNode;

public class RobotRegistrationListener implements
		MessageListener<std_msgs.String> {
	
	private final String DELIMITER = ";";
	
	private Log log;
	private ConsoleController consoleController;

	/**
	 * COnstructor for the RobotRegistrationListener class.
	 * @param connectedNode The ROS node handle 
	 */
	public RobotRegistrationListener(ConnectedNode connectedNode) {
		//Retrieves the node's log and the console in order to
		//record and display status messages
		log = GuiNode.getLog();
		consoleController = ConsoleController.getInstance();

		//Create the subscriber
		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				"hsmrs/robot_configuration", std_msgs.String._TYPE);
		subscriber.addMessageListener(this);
	}

	/**
	 * This method is called whenever a new registration message is received over ROS.
	 * It decodes the message and performs the necessary actions.
	 * @param message The ROS message sent over the topic.
	 */
	@Override
	public void onNewMessage(std_msgs.String message) {
		
		//name;imageTopic;poseTopic
		
		//Split the message along the delimiter
		String[] messageData = message.getData().split(DELIMITER);

		//Begin registration process
		log.info("Confirm registration: " + messageData[0]);
		
		//Create Robot model
		RobotListModel rlm = RobotListModel.getInstance();
		RobotModel newRobot = new RobotModel(messageData[0]);
		newRobot.setImageTopic(messageData[1]);
		newRobot.setImageListener(new ImageListener(messageData[1]));
		newRobot.setPoseListener(new PoseListener(newRobot, messageData[2]));
		rlm.addRobot(newRobot);
		InteractiveMapController.getInstance()
		.updateRobotLocation(newRobot.getName(), 
				new Pair<Integer, Integer>(0, 0));
		
		//End registration process
		log.info("Successful registration: " + messageData[0]);
		long timestamp = System.currentTimeMillis() / 1000;
		consoleController.addLog("System", timestamp, "New agent has been successfully registered: "
						+ messageData[0]);		
	}
}
