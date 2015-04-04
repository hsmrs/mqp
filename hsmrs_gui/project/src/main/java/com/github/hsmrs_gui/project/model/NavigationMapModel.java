package src.main.java.com.github.hsmrs_gui.project.model;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import src.main.java.com.github.hsmrs_gui.project.Globals;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;

public class NavigationMapModel {

	private int height;
	private int width;
	private double resolution;
	private BufferedImage mapImage;
	private double pixelMeterResolution;
	private MapGridCellModel[][] cells;
	private ArrayList<MapGridCellModel> selectedCells;
	private Map<String, MapGridCellModel> objectLocations;
	
	public NavigationMapModel(){
		this(1, 1, 1);
	}
	
	/**
	 * The constructor for the NavigationMapModel.
	 * @param height The number of cells vertically in the map.
	 * @param width The number of cells horizontally in the map. 
	 * @param resolution The number of meters represented by a grid cell [meter/cell].
	 */
	public NavigationMapModel(int height, int width, double resolution){
		this.height = height;
		this.width = width;
		this.resolution = resolution;
		
		cells = new MapGridCellModel[height][width];
		for (int i = 0; i < height; i++){
			for (int j = 0; j < width; j++){
				cells[i][j] = new MapGridCellModel(i, j, 0);
			}
		}
		pixelMeterResolution = 5.5; //pixels per meter. Play with this to make cells bigger/smaller.
		selectedCells = new ArrayList<MapGridCellModel>();
		objectLocations = new HashMap<String, MapGridCellModel>();
	}
	
	/**
	 * The constructor for the NavigationMapModel by background image and pixel to meter resolution.
	 * @param mapImage The image to be shown as the background of the map.
	 * @param pixelMeterResolution The number of pixels which represent a meter [pix/meter].
	 */
	public NavigationMapModel(BufferedImage mapImage, double pixelMeterResolution){
		height = ((int)(((double)mapImage.getHeight()) / pixelMeterResolution));
		width = ((int)(((double)mapImage.getWidth()) / pixelMeterResolution));
				
		cells = new MapGridCellModel[height][width];
		for (int i = 0; i < height; i++){
			for (int j = 0; j < width; j++){
				cells[i][j] = new MapGridCellModel(i, j, 0);
			}
		}
		selectedCells = new ArrayList<MapGridCellModel>();

		this.mapImage = mapImage;
		this.pixelMeterResolution = pixelMeterResolution;
	}
	
	/**
	 * Gets the height of the map.
	 * @return The height of the map.
	 */
	public int getHeight(){
		return height;
	}
	
	/**
	 * Gets the width of the map.
	 * @return The width of the map.
	 */
	public int getWidth(){
		return width;
	}
	
	/**
	 * Gets the size of a grid cell in pixels.
	 * @return The size of a grid cell in pixels.
	 */
	public double getCellSize(){
		return pixelMeterResolution / resolution;
	}
	
	/**
	 * Sets the image to be used as the background for the map.
	 * @param newMapImage The new image to be used as the map background.
	 * @param pixelMeterResolution The number of pixels which represent a meter.
	 * @param redrawGrid True if the grid should be redrawn.
	 */
	public void setMapImage(BufferedImage newMapImage, double pixelMeterResolution, boolean redrawGrid){
		this.mapImage = newMapImage;
		this.pixelMeterResolution = pixelMeterResolution;
		if (redrawGrid){
			this.height = mapImage.getHeight();;
			this.width = mapImage.getWidth();
			
			cells = new MapGridCellModel[height][width];
			for (int i = 0; i < height; i++){
				for (int j = 0; j < width; j++){
					cells[i][j] = new MapGridCellModel(i, j, 0);
				}
			}
		}
	}
	
	/**
	 * Toggle the selected status of the grid cell identified by the parameters.
	 * @param row The row of the cell to be toggled.
	 * @param column The column of the cell to be toggled.
	 */
	public void toggleSelectCell(int row, int column){
		cells[row][column].toggleSelected();
		System.out.println("Toggle select cell at: (" + column + ", " + row + ")");
		try{
			if(cells[row][column].getIsSelected() == true){
				System.out.println("If true");
				Globals.gui_vars.put("mapx", column*resolution);
				Globals.gui_vars.put("mapy", row*resolution);
				selectedCells.add(cells[row][column]);
				refreshSelectedVar();
				return;
			}
			else if ((Integer)Globals.gui_vars.get("mapx") == column
					&&
					(Integer)Globals.gui_vars.get("mapy") == row){
				System.out.println("else if true");
				Globals.gui_vars.put("mapx", null);
				Globals.gui_vars.put("mapy", null);
			}
		}
		catch (NullPointerException e){
			System.out.println("ERROR: mapx or mapy does not have a value!");
		}
		//only called if first if's return is not called.
		selectedCells.remove(cells[row][column]);
		refreshSelectedVar();
	}
	
	/**
	 * Gets a list of cells which are selected.
	 * @return A list of selected cells.
	 */
	public List<MapGridCellModel> getSelectedCells(){
		return selectedCells;
	}
	
	/**
	 * Clears all selected cells.
	 */
	public void clearAllCells(){
		for (MapGridCellModel cell : selectedCells){
			cell.toggleSelected();
		}
		selectedCells.clear();
	}
	
	/**
	 * Applies the given array of values to the cells of this map.
	 * @param values The new values to be applied to the map.
	 */
	public void getNewMapData(int[] values){
		if (values.length != height * width){
			System.out.println("Bad map data, length does not match");
			return;
		}
		
		int i = 0;
		for (int row = 0; row < height; row++){
			for (int col = 0; col < width; col++){
				cells[row][col].setValue(values[i]);
				i++;
			}
		}
	}
	
	/**
	 * Returns the location of an object named in the parameter.
	 * @param targetObject The name of the object whose location is being queried.
	 * @return The location of the named object as a Pair object.
	 */
	public Pair<Integer, Integer> getLocation(String targetObject){
		MapGridCellModel location = objectLocations.get(targetObject);
		int x = location.getColumn();
		int y = location.getRow();
		return new Pair<Integer, Integer>(x, y);
	}
	
	/**
	 * Sets the location of the named object to the given location.
	 * @param targetObject The name of the object whose location is to be set.
	 * @param x The x location of the named object.
	 * @param y The y location of the named object.
	 */
	public void setLocation(String targetObject, int x, int y){
		MapGridCellModel gridCell = cells[y][x];
		objectLocations.put(targetObject, gridCell);		
	}
	
	/**
	 * Refresh the global variable which saves the list of selected cells.
	 */
	public void refreshSelectedVar(){
		StringBuilder strBld = new StringBuilder();
		
		for (MapGridCellModel cell : selectedCells){
			strBld.append("(");
			strBld.append(cell.getColumn()*resolution);
			strBld.append(",");
			strBld.append(cell.getRow()*resolution);
			strBld.append(");");
		}
		
		String result = strBld.toString();
		Globals.gui_vars.put("map_sel", result);
		System.out.println("Selected var: " + result);
	}
}
