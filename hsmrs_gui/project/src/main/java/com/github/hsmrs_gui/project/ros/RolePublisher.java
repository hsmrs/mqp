package src.main.java.com.github.hsmrs_gui.project.ros;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.github.hsmrs_gui.project.GuiNode;

import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RoleModel;

public class RolePublisher {

	private Publisher<hsmrs_framework.RoleMsg> publisher;
	private static RolePublisher instance;
	
	public RolePublisher(){
		ConnectedNode connectedNode = GuiNode.getConnectedNode();
		publisher =
		        connectedNode.newPublisher("hsmrs/role_assign", hsmrs_framework.RoleMsg._TYPE);
	}
	
	public static RolePublisher getInstance(){
		if (instance == null){
			instance = new RolePublisher();
		}
		return instance;
	}
	
	public void publishRole(RoleModel role, RobotModel target){
		hsmrs_framework.RoleMsg msg = role.toMessage();
		msg.getOwners().add(target.getName());
        publisher.publish(msg);
	}
}
