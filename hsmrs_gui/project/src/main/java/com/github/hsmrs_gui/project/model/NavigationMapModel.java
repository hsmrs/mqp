package src.main.java.com.github.hsmrs_gui.project.model;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

public class NavigationMapModel {

	private int height;
	private int width;
	private BufferedImage mapImage;
	private double mapImagePixelMeterResolution;
	private MapGridCellModel[][] cells;
	private ArrayList<MapGridCellModel> selectedCells;
	
	public NavigationMapModel(){
		this(1, 1);
	}
	
	public NavigationMapModel(int height, int width){
		this.height = height;
		this.width = width;
		
		cells = new MapGridCellModel[height][width];
		for (int i = 0; i < height; i++){
			for (int j = 0; j < width; j++){
				cells[i][j] = new MapGridCellModel(i, j, 0);
			}
		}
		selectedCells = new ArrayList<MapGridCellModel>();
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
		this.mapImagePixelMeterResolution = pixelMeterResolution;
	}
	
	public int getHeight(){
		return height;
	}
	
	public int getWidth(){
		return width;
	}
	
	public double getCellSize(){
		return mapImagePixelMeterResolution;
	}
	
	public void setMapImage(BufferedImage newMapImage, double pixelMeterResolution, boolean redrawGrid){
		this.mapImage = newMapImage;
		this.mapImagePixelMeterResolution = pixelMeterResolution;
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
	}
	
	public List<MapGridCellModel> getSelectedCells(){
		return selectedCells;
	}
	
	public void clearAllCells(){
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
}
