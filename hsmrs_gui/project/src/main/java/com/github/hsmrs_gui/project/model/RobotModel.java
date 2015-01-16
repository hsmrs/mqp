/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	RobotModel.java													***
***		This class models a robotic agent. Robotic agents have		***
***		names, assigned tasks, and topics through which images are	***
***		are received.												***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.ros.ImageListener;

public class RobotModel {

	private String name;
	private TaskModel assignedTask;
	private String status;
	private String imageTopic;
	private ImageListener mImageListener;
		
	public RobotModel(){
		name = "No name given";
		assignedTask = new TaskModel();
	}
	
	public RobotModel (String name){
		this.name = name;
		assignedTask = new TaskModel();
	}
	
	public RobotModel (String name, TaskModel task){
		this.name = name;
		assignedTask = task;
	}
	
	public RobotModel (String name, TaskModel task, String imageTopic){
		this.name = name;
		assignedTask = task;
		this.imageTopic = imageTopic;
		mImageListener = new ImageListener(imageTopic);
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

	public void setImageTopic(String imageTopic){
		this.imageTopic = imageTopic;
	}
	
	public void setAssignedTask(TaskModel assignedTask) {
		this.assignedTask = assignedTask;
	}
	
	public void setImageListener(ImageListener il){
		mImageListener = il;
	}
	
	
}
