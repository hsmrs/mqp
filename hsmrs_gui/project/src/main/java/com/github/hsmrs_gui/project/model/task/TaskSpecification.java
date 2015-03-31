package src.main.java.com.github.hsmrs_gui.project.model.task;

import java.util.List;

public class TaskSpecification {
	private String name;
	//parameterTypePairs - Expect string in form: "name:type"
	private List<String> parameterTypePairs;
	
	/**
	 * The constructor for the TaskSpecification class. 
	 * @param name The name of the type of Task this specifies.
	 * @param parameterTypePairs A String encoding a parameter in the following way: "name:type" where name is the name of the parameter and type is the type of the parameter.
	 */
	public TaskSpecification(String name, List<String> parameterTypePairs){
		this.name = name;
		for (String item : parameterTypePairs){
			if(!item.contains(":")){
				throw new IllegalArgumentException("Parameter Type pair: " 
			+ item + " does not match the expected format name:type");
			}
		}
		this.parameterTypePairs = parameterTypePairs;
	}
	
	/**
	 * Gets the name of this TaskSpecification.
	 * @return The name of this TaskSpecification.
	 */
	public String getName(){
		return name;
	}
	
	/**
	 * Get a list of strings encoding the parameters for the task defined by this TaskSpecification
	 * @return The list of String encoded Parameters.
	 */
	public List<String> getParameterTypePairs(){
		return parameterTypePairs;
	}
}
