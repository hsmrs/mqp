/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	RobotModel.java													***
***		This class models a robotic agent. Robotic agents have		***
***		names, assigned tasks, and topics through which images are	***
***		are received.												***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model.robot;

import java.awt.Color;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.ros.HelpListener;
import src.main.java.com.github.hsmrs_gui.project.ros.ImageListener;
import src.main.java.com.github.hsmrs_gui.project.ros.LogListener;
import src.main.java.com.github.hsmrs_gui.project.ros.PoseListener;
import src.main.java.com.github.hsmrs_gui.project.ros.RequestPublisher;
import src.main.java.com.github.hsmrs_gui.project.ros.StatusListener;
import src.main.java.com.github.hsmrs_gui.project.ros.TeleOpPublisher;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;

public class RobotModel {

	private String name;
	private TaskModel assignedTask;
	private String status;
	private RoleModel role;
	private LogListener logListener;
	private String imageTopic;
	private ImageListener imageListener;
	private PoseListener poseListener;
	private StatusListener statusListener;
	private HelpListener helpListener;
	private TeleOpPublisher teleOpPublisher;
	private RequestPublisher requestPublisher;
	private boolean needsHelp;
	private Color color;
	private Pair<Integer, Integer> location;
	
	/**
	 * An empty constructor for the RobotModel class.
	 */
	public RobotModel(){
		name = "No name given";
		status = "Connected";
		assignedTask = new TaskModel();
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	/**
	 * A constructor for the RobotModel class by name.
	 */
	public RobotModel (String name){
		this.name = name;
		status = "Connected";
		assignedTask = new TaskModel();
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	/**
	 * A constructor for the RobotModel class by name and task.
	 */
	public RobotModel (String name, TaskModel task){
		this.name = name;
		status = "Connected";
		assignedTask = task;
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	/**
	 * A constructor for the RobotModel class by name, task, and by the topic to listen for video feeds from.
	 */
	public RobotModel (String name, TaskModel task, String imageTopic){
		this.name = name;
		status = "Connected";
		assignedTask = task;
		this.imageTopic = imageTopic;
		imageListener = new ImageListener(imageTopic);
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}

	/**
	 * Returns the name of this RobotModel.
	 * @return The name of this RobotModel.
	 */
	public String getName() {
		return name;
	}

	/**
	 * Sets the name of this RobotModel.
	 * @param name The new name for this RobotModel.
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Get the task owned by this robot.
	 * @return The TaskModel owned by this RobotModel.
	 */
	public TaskModel getAssignedTask() {
		return assignedTask;
	}
	
	/**
	 * Gets the status of this robot.
	 * @return The status of this robot.
	 */
	public String getStatus(){
		return status;
	}
	
	/**
	 * Gets the Role assigned to this robot.
	 * @return The role assigned to this robot.
	 */
	public RoleModel getRole(){
		return role;
	}
	
	/**
	 * Gets the ROS subscriber for log messages from this robot.
	 * @return The ROS subscriber LogListener associated with this robot.
	 */
	public LogListener getLogListener(){
		return logListener;
	}
	
	/**
	 * Gets the ROS topic for images sent from this robot.
	 * @return The ROS topic for images sent from this robot.
	 */
	public String getImageTopic(){
		return imageTopic;
	}
	
	/**
	 * Gets the ROS subscriber for pose messages from this robot.
	 * @return The ROS subscriber PoseListener associated with this robot.
	 */
	public PoseListener getPoseListener(){
		return poseListener;
	}
	
	/**
	 * Gets the ROS subscriber for status messages from this robot.
	 * @return The ROS subscriber StatusListener associated with this robot.
	 */
	public StatusListener getStatusListener(){
		return statusListener;
	}
	
	/**
	 * Gets the ROS subscriber for help messages from this robot.
	 * @return The ROS subscriber HelpListener associated with this robot.
	 */
	public HelpListener getHelpListener(){
		return helpListener;
	}
	
	/**
	 * Returns true if the robot's help status is true.
	 * @return True if the robot's help status is true.
	 */
	public boolean getNeedsHelp(){
		return needsHelp;
	}
	
	/**
	 * Returns the color associated with this robot.
	 * @return The color associated with this robot.
	 */
	public Color getColor(){
		return color;
	}
	
	/**
	 * Returns the location of this robot as a Pair type
	 * @return The location of this robot as a Pair type
	 */
	public Pair<Integer, Integer> getLocation(){
		return location;
	}
	
	/**
	 * Returns the ROS publisher class for the Tele-Op commands
	 * @return The ROS publisher class for the Tele-Op commands
	 */
	public TeleOpPublisher getTeleOpPublisher(){
		return teleOpPublisher;
	}
	
	/**
	 * Sets the ROS subscriber for log messages from this robot.
	 * @param The ROS subscriber to be associated with this Robot.
	 */
	public void setLogListener(LogListener logListener) {
		this.logListener = logListener;
	}

	/**
	 * Sets the topic for the robot's image listener to subscribe to.
	 * @param imageTopic The topic to subscribe to for images.
	 */
	public void setImageTopic(String imageTopic){
		this.imageTopic = imageTopic;
	}
	
	/**
	 * Sets the task which this robot owns.
	 * @param assignedTask The task that this robot owns.
	 */
	public void setAssignedTask(TaskModel assignedTask) {
		this.assignedTask = assignedTask;
	}
	
	/**
	 * Set the status of this robot
	 * @param newStatus The new status of this robot.
	 */
	public void setStatus(String newStatus){
		this.status = newStatus;
	}
	
	/**
	 * Set the Role assigned to this Robot
	 * @param role The role to assign to this robot
	 */
	public void setRole(RoleModel role){
		this.role = role;
	}
	
	/**
	 * Set the ROS listener for images
	 * @param il the image listener for images
	 */
	public void setImageListener(ImageListener il){
		imageListener = il;
	}
	
	/**
	 * Set the ROS listener for Poses
	 * @param poseListener 
	 */
	public void setPoseListener(PoseListener poseListener) {
		this.poseListener = poseListener;
	}
	
	/**
	 * Set the ROS listener for Status messages
	 * @param statusListener The listener for status messages
	 */
	public void setStatusListener(StatusListener statusListener){
		this.statusListener = statusListener;
	}
	
	/**
	 * Set the ROS listener for help messages
	 * @param helpListener The listener for help messages
	 */
	public void setHelpListener(HelpListener helpListener){
		this.helpListener = helpListener;
	}
	
	/**
	 * Sets whether or not this robot needs help
	 * @param needsHelp True if this robot needs help
	 */
	public void setNeedsHelp(boolean needsHelp){
		this.needsHelp = needsHelp;
	}
	
	/**
	 * Sets the location of this robot
	 * @param location The location of this robot as a Pair data type.
	 */
	public void setLocation(Pair<Integer, Integer> location){
		this.location = location;
	}
	
	/**
	 * Sets the ROS publisher for Tele-Op commands
	 * @param publisher
	 */
	public void setTeleOpPublisher(TeleOpPublisher publisher){
		teleOpPublisher = publisher;
	}

	/**
	 * Accepts a tele-op command as a string and delegates sending the message to the
	 * tele-op publisher associated with this robot.
	 * @param direction The direction of the tele-op command
	 */
	public void sendTeleOpCommand(String direction) {
		teleOpPublisher.publishMessage(direction);
	}

	/**
	 * Gets the publisher for requests associated with this robot.
	 * @return
	 */
	public RequestPublisher getRequestPublisher() {
		return requestPublisher;
	}
	
	/**
	 * Sets the request publisher associated with this robot
	 * @param requestPublisher The request publisher to associate with this robot.
	 */
	public void setRequestPublisher(RequestPublisher requestPublisher) {
		this.requestPublisher = requestPublisher;
	}
}
