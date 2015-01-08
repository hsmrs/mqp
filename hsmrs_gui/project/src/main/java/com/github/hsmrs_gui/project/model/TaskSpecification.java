package src.main.java.com.github.hsmrs_gui.project.model;

import java.util.List;

public class TaskSpecification {
	private String name;
	//parameterTypePairs - Expect string in form: "name:type"
	private List<String> parameterTypePairs;
	
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
	
	public String getName(){
		return name;
	}
	
	public List<String> getParameterTypePairs(){
		return parameterTypePairs;
	}
}
