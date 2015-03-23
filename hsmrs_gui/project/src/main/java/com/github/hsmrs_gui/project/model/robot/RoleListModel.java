package src.main.java.com.github.hsmrs_gui.project.model.robot;

import java.util.ArrayList;
import java.util.List;

public class RoleListModel {

	private static RoleListModel instance;
	private ArrayList<RoleModel> roleList;
	
	/**
	 * The constructor for the RoleListModel class
	 */
	private RoleListModel(){
		roleList = new ArrayList<RoleModel>();
	}
	
	/**
	 * Returns the only existing instance of RoleListModel.
	 * @return The only existing instance of RoleListModel.
	 */
	public static RoleListModel getInstance(){
		if (instance == null){
			instance = new RoleListModel();
		}
		
		return instance;
	}
	
	/**
	 * Adds a role to the list of defined roles.
	 * @param newRole The role to be added to the list.
	 */
	public void addRole(RoleModel newRole){
		roleList.add(newRole);
	}
	
	/**
	 * Removes a role from the list of defined roles.
	 * @param role The role to be removed.
	 */
	public void removeRole(RoleModel role){
		roleList.remove(role);
	}
	
	/**
	 * Gets a RoleModel by its name.
	 * @param name The name of the role.
	 * @return The role with the given name.
	 */
	public RoleModel getRoleByName(String name){
		for (RoleModel rm : roleList){
			if (rm.getType().equals(name)){
				return rm;
			}
		}
		return null;
	}
	
	/**
	 * Gets a list of all of the defined roles.
	 * @return The list of all defined roles.
	 */
	public List<String> getRoleNames(){
		ArrayList<String> returnList = new ArrayList<String>();
		for (RoleModel rm : roleList){
			returnList.add(rm.getType());
		}
		
		return returnList;
	}

	/**
	 * Gets the role in the given index.
	 * @param index The index of the role.
	 * @return The RoleModel at the given index.
	 */
	public RoleModel getRole(int index) {
		return roleList.get(index);
	}
}
