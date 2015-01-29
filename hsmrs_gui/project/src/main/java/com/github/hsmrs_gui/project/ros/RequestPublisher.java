package src.main.java.com.github.hsmrs_gui.project.ros;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.github.hsmrs_gui.project.GuiNode;

public class RequestPublisher {
	
	private final String TELE_OP_REQUEST = "tele-op";
	private final String STOP_TELE_OP_REQUEST = "stop tele-op";

	private Publisher<std_msgs.String> publisher;
	private ConnectedNode connectedNode;
	
	public RequestPublisher(String topicName){
		connectedNode = GuiNode.getConnectedNode();
		
		publisher = connectedNode.newPublisher(topicName, std_msgs.String._TYPE);
	}
	
	public void publishTeleOpRequest(){
		publishMessage(TELE_OP_REQUEST);
	}
	
	public void publishStopTeleOpRequest(){
		publishMessage(STOP_TELE_OP_REQUEST);
	}
	
	private void publishMessage(String text){
		System.out.println("Request publish: " + text);
		std_msgs.String str = publisher.newMessage();
        str.setData(text);
        publisher.publish(str);
	}
}
