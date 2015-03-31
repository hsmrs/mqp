/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	MapListener.java												***
***		This class defines a ROS subscriber for Map messages. When	***
***		a new map is heard through ROS, the map is converted into	***
***		a format understood by the GUI and passed to the			***
***		InteractiveMapController object.							***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.ros;

import org.ros.message.MessageListener;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;

import src.main.java.com.github.hsmrs_gui.project.controller.InteractiveMapController;

import com.github.hsmrs_gui.project.GuiNode;

public class MapListener implements MessageListener<nav_msgs.OccupancyGrid> {

	/**
	 * Constructor for the MapListener class. All MapListeners require
	 * a reference to the ConnectedNode, i.e. the ROS handle.
	 * @param connectedNode
	 */
	public MapListener(ConnectedNode connectedNode){
		Subscriber<nav_msgs.OccupancyGrid> subscriber = connectedNode.newSubscriber(
				"hsmrs/navigation_map", nav_msgs.OccupancyGrid._TYPE);
		subscriber.addMessageListener(this);
	}
	
	/**
	 * This method is called whenever a new ROS OccupancyGrid message is received. Any function
	 * called from this method should be declared synchronized.
	 * @param grid The OccupancyGrid message heard from the topic.
	 */
	@Override
	public void onNewMessage(nav_msgs.OccupancyGrid grid) {
		InteractiveMapController imc = InteractiveMapController.getInstance();
		ChannelBuffer gridData = grid.getData();
		int gridHeight = grid.getInfo().getHeight();
		int gridWidth = grid.getInfo().getWidth();
		int[] data = new int[gridHeight * gridWidth];
		
		int i = 0;
		 while (gridData.readable()) {
		     data[i] = gridData.readByte();
		     i++;
		 }
		imc.getNewMapData(data);
	}
}
