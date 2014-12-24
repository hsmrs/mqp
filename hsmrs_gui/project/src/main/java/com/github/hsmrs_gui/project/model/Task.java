/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	Task.java														***
***		This class models a task. A task has a name, a list of		***
***		owners, a list of subtasks, and a status. 					***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model;

import java.util.ArrayList;
import java.util.List;

public class Task {

	private String name;
	private List<RobotModel> owners;
	private String status;
	
	public Task(){
		name = "Idle";
		owners = new ArrayList<RobotModel>();
		status = "---";
	}
	
	public Task(String name){
		this.name = name;
		owners = new ArrayList<RobotModel>();
		status = "Not claimed";
	}
	
	public Task(String name, List<RobotModel> owners, String status){
		this.name = name;
		this.owners = owners;
		this.status = status;
	}
	
	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public List<RobotModel> getOwners() {
		return owners;
	}

	public void addOwner(RobotModel owner) {
		owners.add(owner);
	}
	
	public void removeOwner(RobotModel owner){
		owners.remove(owner);
	}

	public String getStatus() {
		return status;
	}

	public void setStatus(String status) {
		this.status = status;
	}

	public String toString(){
		return name;
	}
}
