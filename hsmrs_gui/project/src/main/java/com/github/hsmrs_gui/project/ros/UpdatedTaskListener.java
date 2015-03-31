package src.main.java.com.github.hsmrs_gui.project.ros;

import java.util.ArrayList;
import java.util.List;

import com.github.hsmrs_gui.project.GuiNode;

import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;
import org.ros.message.MessageListener;
import org.apache.commons.logging.Log;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;
import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskPanel;

public class UpdatedTaskListener implements MessageListener<hsmrs_framework.TaskMsg>{

	private ConnectedNode connectedNode;
	TaskListModel tasks;
	
	/**
	 * The constructor for the UpdatedTaskListener class.
	 * @param topicName The name of the ROS topic
	 */
	public UpdatedTaskListener() {
		connectedNode = GuiNode.getConnectedNode();
		this.tasks = TaskListModel.getInstance();

		Subscriber<hsmrs_framework.TaskMsg> subscriber = connectedNode.newSubscriber(
				"hsmrs/updated_task", hsmrs_framework.TaskMsg._TYPE);
		subscriber.addMessageListener(this);
	}

	/**
	 * This method is called whenever a message is received through ROS.
	 * @param message The String message received through the topic.
	 */
	@Override
	public void onNewMessage(hsmrs_framework.TaskMsg message) {
		if (message.getStatus().equals("deleted")) return;
		
		TaskModel task = tasks.getTaskByID((int)message.getId());
		List<String> msgOwners = message.getOwners();
		List<RobotModel> oldOwners = task.getOwners();
		
		for (RobotModel owner : oldOwners){
			task.removeOwner(owner);
		}
		for (String ownerName : msgOwners){
			task.addOwner(
					RobotListModel.getInstance().
					getRobotModelByName(ownerName));
		}
		
		task.setStatus(message.getStatus());
		
		TaskPanel.getInstance().updateTaskList();
	}

}
