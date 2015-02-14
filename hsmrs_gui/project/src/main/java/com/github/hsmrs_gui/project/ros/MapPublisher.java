package src.main.java.com.github.hsmrs_gui.project.ros;

import java.nio.ByteOrder;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

import com.github.hsmrs_gui.project.GuiNode;

public class MapPublisher {
	
	private ConnectedNode connectedNode;

	private Publisher<nav_msgs.OccupancyGrid> publisher;
	private static MapPublisher instance;
	
	public MapPublisher(String topicName){
		connectedNode = GuiNode.getConnectedNode();
		
		publisher =
		        connectedNode.newPublisher(topicName, nav_msgs.OccupancyGrid._TYPE);
		publisher.setLatchMode(true);
		instance = this;
	}
	
	public static MapPublisher getInstance(){
		return instance;
	}
	
	public void publishMap(int height, int width, float resolution){
		nav_msgs.OccupancyGrid map = publisher.newMessage();
		map.getInfo().setHeight(height);
		map.getInfo().setWidth(width);
		map.getInfo().setResolution(resolution);
		ChannelBuffer data = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, new byte[height*width]);
		//ChannelBuffer data = ChannelBuffers.buffer(height * width);
		map.setData(data);
		
        publisher.publish(map);
	}
}
