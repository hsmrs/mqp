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
	
		
	public RobotModel(){
		name = "No name given";
		status = "Connected";
		assignedTask = new TaskModel();
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	public RobotModel (String name){
		this.name = name;
		status = "Connected";
		assignedTask = new TaskModel();
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	public RobotModel (String name, TaskModel task){
		this.name = name;
		status = "Connected";
		assignedTask = task;
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	public RobotModel (String name, TaskModel task, String imageTopic){
		this.name = name;
		status = "Connected";
		assignedTask = task;
		this.imageTopic = imageTopic;
		imageListener = new ImageListener(imageTopic);
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public TaskModel getAssignedTask() {
		return assignedTask;
	}
	
	public String getStatus(){
		return status;
	}
	
	public RoleModel getRole(){
		return role;
	}
	
	public LogListener getLogListener(){
		return logListener;
	}
	
	public String getImageTopic(){
		return imageTopic;
	}
	
	public PoseListener getPoseListener(){
		return poseListener;
	}
	
	public StatusListener getStatusListener(){
		return statusListener;
	}
	
	public HelpListener getHelpListener(){
		return helpListener;
	}
	
	public boolean getNeedsHelp(){
		return needsHelp;
	}
	
	public Color getColor(){
		return color;
	}
	
	public Pair<Integer, Integer> getLocation(){
		return location;
	}
	
	public TeleOpPublisher getTeleOpPublisher(){
		return teleOpPublisher;
	}
	
	public void setLogListener(LogListener logListener) {
		this.logListener = logListener;
	}

	public void setImageTopic(String imageTopic){
		this.imageTopic = imageTopic;
	}
	
	public void setAssignedTask(TaskModel assignedTask) {
		this.assignedTask = assignedTask;
	}
	
	public void setStatus(String newStatus){
		this.status = newStatus;
	}
	
	public void setRole(RoleModel role){
		this.role = role;
	}
	
	public void setImageListener(ImageListener il){
		imageListener = il;
	}
	
	public void setPoseListener(PoseListener poseListener) {
		this.poseListener = poseListener;
	}
	
	public void setStatusListener(StatusListener statusListener){
		this.statusListener = statusListener;
	}
	
	public void setHelpListener(HelpListener helpListener){
		this.helpListener = helpListener;
	}
	
	public void setNeedsHelp(boolean needsHelp){
		this.needsHelp = needsHelp;
	}
	
	public void setLocation(Pair<Integer, Integer> location){
		this.location = location;
	}
	
	public void setTeleOpPublisher(TeleOpPublisher publisher){
		teleOpPublisher = publisher;
	}

	public void sendTeleOpCommand(String direction) {
		teleOpPublisher.publishMessage(direction);
	}

	public RequestPublisher getRequestPublisher() {
		return requestPublisher;
	}
	
	public void setRequestPublisher(RequestPublisher requestPublisher) {
		this.requestPublisher = requestPublisher;
	}
}
