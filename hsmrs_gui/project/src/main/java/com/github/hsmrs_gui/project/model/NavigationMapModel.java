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
		pixelMeterResolution = 13;
		selectedCells = new ArrayList<MapGridCellModel>();
		objectLocations = new HashMap<String, MapGridCellModel>();
	}
	
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
	
	public int getHeight(){
		return height;
	}
	
	public int getWidth(){
		return width;
	}
	
	public double getCellSize(){
		return pixelMeterResolution / resolution;
	}
	
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
	
	public void toggleSelectCell(int row, int column){
		cells[row][column].toggleSelected();
		System.out.println("Toggle select cell at: (" + column + ", " + row + ")");
		try{
			if(cells[row][column].getIsSelected() == true){
				System.out.println("If true");
				Globals.gui_vars.put("mapx", column);
				Globals.gui_vars.put("mapy", row);
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
	
	public List<MapGridCellModel> getSelectedCells(){
		return selectedCells;
	}
	
	public void clearAllCells(){
		for (MapGridCellModel cell : selectedCells){
			cell.toggleSelected();
		}
		selectedCells.clear();
	}
	
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
	
	public Pair<Integer, Integer> getLocation(String targetObject){
		MapGridCellModel location = objectLocations.get(targetObject);
		int x = location.getColumn();
		int y = location.getRow();
		return new Pair<Integer, Integer>(x, y);
	}
	
	public void setLocation(String targetObject, int x, int y){
		MapGridCellModel gridCell = cells[y][x];
		objectLocations.put(targetObject, gridCell);		
	}
	
	public void refreshSelectedVar(){
		StringBuilder strBld = new StringBuilder();
		
		for (MapGridCellModel cell : selectedCells){
			strBld.append("(");
			strBld.append(cell.getColumn());
			strBld.append(",");
			strBld.append(cell.getRow());
			strBld.append(");");
		}
		
		String result = strBld.toString();
		Globals.gui_vars.put("map_selected", result);
		System.out.println("Selected var: " + result);
	}
}
