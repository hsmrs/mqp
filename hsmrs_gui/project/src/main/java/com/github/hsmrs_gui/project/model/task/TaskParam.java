package src.main.java.com.github.hsmrs_gui.project.model.task;

public class TaskParam <T>{
	String paramLabel;
	T value;
	
	/**
	 * The constructor for the TaskParam class.
	 * @param label The label for this TaskParam.
	 * @param value The value of this TaskParam.
	 */
	public TaskParam(String label, T value){
		this.paramLabel = label;
		this.value = value;
	}
	
	/**
	 * Gets the label for this TaskParam.
	 * @return The label for this TaskParam.
	 */
	public String getLabel(){
		return paramLabel;
	}
	
	/**
	 * Gets the value of this TaskParam.
	 * @return
	 */
	public T getValue(){
		return value;
	}
	
	/**
	 * Gets the type of this TaskParam.
	 * @return The type of this TaskParam as a String.
	 */
	public String getType(){
		if (value instanceof Integer){
			return "Integer";
		}
		else if (value instanceof Double){
			return "Double";
		}
		else if (value instanceof String){
			return "String";
		}
		else {
			return "Unknown";
		}
	}
	
	/**
	 * Sets the value of this TaskParam.
	 * @param value The new value for this TaskParam.
	 */
	public void setValue(T value){
		this.value = value;
	}
}
