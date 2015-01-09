package src.main.java.com.github.hsmrs_gui.project.model.task;

public class TaskParam <T>{
	String paramLabel;
	T value;
	
	public TaskParam(String label, T value){
		this.paramLabel = label;
		this.value = value;
	}
	
	public String getLabel(){
		return paramLabel;
	}
	
	public T getValue(){
		return value;
	}
	
	public void setValue(T value){
		this.value = value;
	}
}
