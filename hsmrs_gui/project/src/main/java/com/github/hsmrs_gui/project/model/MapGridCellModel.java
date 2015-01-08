package src.main.java.com.github.hsmrs_gui.project.model;

public class MapGridCellModel {
	private int row;
	private int column;
	private int value;
	private boolean isSelected = false;
	
	public MapGridCellModel(int row, int column, int value){
		this.row = row;
		this.column = column;
		this.value = value;
	}
	
	public void setValue(int newValue){
		this.value = newValue;
	}
	
	public void setSelected(boolean isSelected){
		this.isSelected = isSelected;
	}
	
	public void toggleSelected(){
		isSelected = (!isSelected);
	}
	
	public int getValue(){
		return value;
	}
	
	public boolean getIsSelected(){
		return isSelected;
	}
}
