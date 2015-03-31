package src.main.java.com.github.hsmrs_gui.project.model;

import src.main.java.com.github.hsmrs_gui.project.util.Colors;

public class MapGridCellModel {
	private int row;
	private int column;
	private int value;
	private boolean isSelected = false;
	
	/**
	 * The constructor for the MapGridCellModel class.
	 * @param row The row of this cell
	 * @param column The column of this class.
	 * @param value The value of this cell.
	 */
	public MapGridCellModel(int row, int column, int value){
		this.row = row;
		this.column = column;
		this.value = value;
	}
	
	/**
	 * Sets the value of this grid cell.
	 * @param newValue The new value of this grid cell.
	 */
	public void setValue(int newValue){
		this.value = newValue;
	}
	
	/**
	 * Sets the selected status of this grid cell.
	 * @param isSelected True if this grid cell is selected.
	 */
	public void setSelected(boolean isSelected){
		this.isSelected = isSelected;
	}
	
	/**
	 * Toggles the selected status of this grid cell.
	 */
	public void toggleSelected(){
		isSelected = (!isSelected);
	}
	
	/**
	 * Get the value of this grid cell.
	 * @return The value of this grid cell.
	 */
	public int getValue(){
		return value;
	}
	
	/**
	 * Gets the selected status of this grid cell.
	 * @return True if this grid cell is selected.
	 */
	public boolean getIsSelected(){
		return isSelected;
	}
	
	/**
	 * Gets the row of this grid cell.
	 * @return The row of this grid cell.
	 */
	public int getRow(){
		return row;
	}
	
	/**
	 * Gets the column of this grid cell.
	 * @return The column of this grid cell.
	 */
	public int getColumn(){
		return column;
	}
}
