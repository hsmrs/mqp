package src.main.java.com.github.hsmrs_gui.project.model.task;

public class PrerequisteModel {
	
	private boolean isFulfilled = false;

	/**
	 * Constructor for the PrerequisiteModel class.
	 */
	public PrerequisteModel(){
		
	}
	
	/**
	 * Returns true if the prerequsite is fulfilled.
	 * @return True if the Prerequisite is fulfilled.
	 */
	public boolean isFulfilled(){
		return isFulfilled;
	}
}
