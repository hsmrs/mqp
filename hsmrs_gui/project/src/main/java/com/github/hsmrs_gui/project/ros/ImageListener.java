/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	ImageListener.java												***
***		This class defines a ROS subscriber for Image messages.		***
***		Image messages are converted into BufferedImages and passed	***
***		to the VideoController.										***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.ros;
//import static com.googlecode.javacv.cpp.opencv_imgproc.CV_THRESH_BINARY;
//import static com.googlecode.javacv.cpp.opencv_imgproc.cvThreshold;

import java.awt.image.BufferedImage;
import java.awt.image.ComponentSampleModel;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferByte;
import java.awt.image.Raster;
import java.awt.image.SampleModel;
import java.awt.image.WritableRaster;
import java.io.ByteArrayInputStream;

import javax.imageio.ImageIO;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import org.ros.node.ConnectedNode;

import com.github.hsmrs_gui.project.GuiNode;

import org.jboss.netty.buffer.ChannelBuffer;

import src.main.java.com.github.hsmrs_gui.project.controller.VideoController;
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotListView;

public class ImageListener implements MessageListener<sensor_msgs.Image> {
	long time;
	boolean isDefault;
	String topicName;

	/**
	 * Constructor for the ImageListener class. All ImageListeners require
	 * a reference to the ConnectedNode, i.e. the ROS handle. The topic that
	 * the ImageListener created through this constructor listens to is the default.
	 * @param connectedNode
	 */
	public ImageListener(ConnectedNode connectedNode){
		Subscriber<sensor_msgs.Image> subscriber = connectedNode.newSubscriber(
				"hsmrs/test_images", sensor_msgs.Image._TYPE);
		subscriber.addMessageListener(this);
		topicName = "Default";
		isDefault = true;
	}
	
	/**
	 * Constructor for the ImageListener class. An ImageListener created through
	 * this constructor listens to the ROS topic specified by the parameter.
	 * @param topicName
	 */
	public ImageListener(String topicName){
		Subscriber<sensor_msgs.Image> subscriber = GuiNode.getConnectedNode().newSubscriber(
				topicName, sensor_msgs.Image._TYPE);
		subscriber.addMessageListener(this);
		isDefault = false;
		this.topicName = topicName;
	}
	
	/**
	 * This method is called whenever a new ROS Image message is received. Any function
	 * called from this method should be declared synchronized.
	 * @param img The Image message heard from the topic.
	 */
	@Override
	public void onNewMessage(sensor_msgs.Image img) {
		time = System.currentTimeMillis();
		String cameraName = img.getHeader().getFrameId();
		
		//Has the robot list view been created yet?
		boolean robotListViewExists = RobotListView.getInstance().getRobotInFocus() != null;
		
		//Is there a robot currently in focus?
		boolean robotInFocus = robotListViewExists &&
				RobotListView.getInstance().getRobotInFocus().getName() != null;
		
//		if (robotInFocus) For debugging
//			System.out.println(topicName + " --- " + RobotListView.getInstance().getRobotInFocus().getImageTopic());
		
		//If the image is not coming from the robot in focus, discard it.
		if (robotInFocus
				&&
				!topicName.equals(RobotListView.getInstance().getRobotInFocus().getImageTopic()))
			return;
		else if ((!robotInFocus && isDefault) 
				|| 
				(robotInFocus && topicName.equals(RobotListView.getInstance().getRobotInFocus().getImageTopic()))){

			//If the image is coming from the correct source
			BufferedImage buffImg = messageToBufferedImage(img);
			if (buffImg == null){
				GuiNode.getLog().info("Bad image read");
				return;
			}
			//Send converted image to the VideoController.
			VideoController.getInstance().receiveNewImage(cameraName, buffImg);
		}

	}
	
	/**
	 * Converts an Image message to a BufferedImage.
	 * @param imgMsg The raw Image message.
	 * @return A BufferedImage converted from a ROS Image message.
	 */
	public static BufferedImage messageToBufferedImage(sensor_msgs.Image imgMsg){
		int width = (int) imgMsg.getWidth();
		int height = (int) imgMsg.getHeight();	
		
		//Perform the conversion
		ChannelBuffer data = imgMsg.getData();
		int imageSize = width * height * 3;
		int[] pixels = new int[imageSize];
		int i = 0;
		 while (data.readable()) {
		     pixels[i] = data.readByte();
		     i++;
		 }
		 
		 BufferedImage readImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
		 WritableRaster raster = readImage.getRaster();
	     raster.setPixels(0, 0, width, height, pixels);
	     return readImage;
		 
	}
}
