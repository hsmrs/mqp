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
	
	/**
	 * The constructor for the MapPublisher class.
	 * @param topicName The topic to publish OccupancyGrid messages to.
	 */
	public MapPublisher(String topicName){
		connectedNode = GuiNode.getConnectedNode();
		
		publisher =
		        connectedNode.newPublisher(topicName, nav_msgs.OccupancyGrid._TYPE);
		publisher.setLatchMode(true);
		instance = this;
	}
	
	/**
	 * Gets the instance of the MapPublisher.
	 * @return The instance of the MapPublisher.
	 */
	public static MapPublisher getInstance(){
		return instance;
	}
	
	/**
	 * Publishes an empty OccupancyGrid ROS message with the given specifications.
	 * @param height The height of the OccupancyGrid.
	 * @param width The width of the OccupancyGrid.
	 * @param resolution The resolution of the OccupancyGrid.
	 */
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
