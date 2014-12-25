/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	SystemLogListener.java									***
***		This class defines a ROS subscriber which listens for		***
***		String messages which carry messages about the HSMRS system.***
***		The messages are logged using the node's log and using the	***
***		GUI console.												***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.ros;

import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
import org.apache.commons.logging.Log;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;

import com.github.hsmrs_gui.project.GuiNode;

public class SystemLogListener implements MessageListener<std_msgs.String>{

	private Log log;
	private ConsoleController consoleController;

	/**
	 * The constructor for the SystemLogListener class.
	 * @param connectedNode The ROS node handle
	 */
	public SystemLogListener(ConnectedNode connectedNode) {
		log = GuiNode.getLog();
		consoleController = ConsoleController.getInstance();

		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				"hsmrs/system_log", std_msgs.String._TYPE);
		subscriber.addMessageListener(this);
	}

	/**
	 * This method is called whenever a message is received through ROS.
	 * Messages are written to the node's log and to the GUI console in
	 * the System channel.
	 * @param message The String message received through the topic.
	 */
	@Override
	public void onNewMessage(std_msgs.String message) {
		log.info("Confirm system log: " + message.getData());
		long timestamp = System.currentTimeMillis() / 1000;
		consoleController.addLog("System", timestamp, message.getData());
	}
}
