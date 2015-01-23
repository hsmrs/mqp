package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.util.ArrayList;
import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.RoleModel;

public class RoleListModel {

	private static RoleListModel instance;
	private ArrayList<RoleModel> roleList;
	
	private RoleListModel(){
		roleList = new ArrayList<RoleModel>();
	}
	
	public static RoleListModel getInstance(){
		if (instance == null){
			instance = new RoleListModel();
		}
		
		return instance;
	}
	
	public void addRole(RoleModel newRole){
		roleList.add(newRole);
	}
	
	public void removeRole(RoleModel role){
		roleList.remove(role);
	}
	
	public RoleModel getRoleByName(String name){
		for (RoleModel rm : roleList){
			if (rm.getName().equals(name)){
				return rm;
			}
		}
		return null;
	}
	
	public List<String> getRoleNames(){
		ArrayList<String> returnList = new ArrayList<String>();
		for (RoleModel rm : roleList){
			returnList.add(rm.getName());
		}
		
		return returnList;
	}

	public RoleModel getRole(int selectedIndex) {
		return roleList.get(selectedIndex);
	}
}
