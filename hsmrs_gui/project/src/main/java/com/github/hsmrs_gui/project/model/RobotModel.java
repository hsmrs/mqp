/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	RobotModel.java													***
***		This class models a robotic agent. Robotic agents have		***
***		names, assigned tasks, and topics through which images are	***
***		are received.												***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model;

import java.awt.Color;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.ros.ImageListener;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;

public class RobotModel {

	private String name;
	private TaskModel assignedTask;
	private String status;
	private String imageTopic;
	private ImageListener mImageListener;
	private Color color;
	private Pair<Integer, Integer> location;
		
	public RobotModel(){
		name = "No name given";
		assignedTask = new TaskModel();
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	public RobotModel (String name){
		this.name = name;
		assignedTask = new TaskModel();
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	public RobotModel (String name, TaskModel task){
		this.name = name;
		assignedTask = task;
		color = Colors.chooseRobotColor();
		location = new Pair<Integer, Integer>(5, 5);
	}
	
	public RobotModel (String name, TaskModel task, String imageTopic){
		this.name = name;
		assignedTask = task;
		this.imageTopic = imageTopic;
		mImageListener = new ImageListener(imageTopic);
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
	
	public String getImageTopic(){
		return imageTopic;
	}
	
	public Color getColor(){
		return color;
	}
	
	public Pair<Integer, Integer> getLocation(){
		return location;
	}

	public void setImageTopic(String imageTopic){
		this.imageTopic = imageTopic;
	}
	
	public void setAssignedTask(TaskModel assignedTask) {
		this.assignedTask = assignedTask;
	}
	
	public void setImageListener(ImageListener il){
		mImageListener = il;
	}
	
	public void setLocation(Pair<Integer, Integer> location){
		this.location = location;
	}
}
