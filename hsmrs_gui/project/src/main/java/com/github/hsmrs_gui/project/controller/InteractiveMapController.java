/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	InteractiveMapController.java									***
***		This class defines the controller which provides all of the	***
***		logical functionality of the interactive map module.  It	***
***		listens to mouse clicks and modifies the map accordingly.	***
***		It then delegates changes to the graphical map to the		***
***		InteractiveMapView class.	This class folows the Singleton	***
***		design pattern.												***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.controller;

import java.awt.Color;
import java.awt.Component;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JLabel;

import src.main.java.com.github.hsmrs_gui.project.model.NavigationMapModel;
import src.main.java.com.github.hsmrs_gui.project.ros.MapPublisher;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;
import src.main.java.com.github.hsmrs_gui.project.view.situational_awareness.InteractiveMapViewLayered;


public class InteractiveMapController implements MouseListener, MouseMotionListener{

	private static InteractiveMapController instance;
	private NavigationMapModel navMapModel;
	private InteractiveMapViewLayered navMapView;
	private ArrayList<JLabel> highlightedCells;
	private ConsoleController consoleController;
	private boolean isDragging = false;
	
	private String INSTITUTE_PARK_MAP_PATH = "";
	private final double INSTITUTE_PARK_MAP_SCALE = 6.3876889849; //pixels per meter
	private BufferedImage mapImage;
	
	private int defaultHeight = 48;
	private int defaultWidth = 48;
	private double defaultResolution = 0.5;
	
	private MapPublisher mapPublisher;
		
	/**
	 * Constructor for the InteractiveMapController class
	 */
	private InteractiveMapController(){
		consoleController = ConsoleController.getInstance();
		highlightedCells = new ArrayList<JLabel>();
		
		INSTITUTE_PARK_MAP_PATH = String.format("%s/project/%s", System.getProperty("user.dir"), this.getClass().getPackage().getName().replace(".", "/"));
		INSTITUTE_PARK_MAP_PATH += "/images/institute_park_edited.jpg";
		
		/*mapImage = null;
		try{
			mapImage = ImageIO.read(new File(INSTITUTE_PARK_MAP_PATH));
			navMapModel = new NavigationMapModel(mapImage, INSTITUTE_PARK_MAP_SCALE);
			resolution = 1;
		}
		catch(Exception e){
			System.out.println("Error reading image for interactive map image");
			navMapModel = new NavigationMapModel(defaultHeight, defaultWidth);
			resolution = 0.5
		}*/
		mapPublisher = new MapPublisher("hsmrs/map");
		navMapModel = new NavigationMapModel(defaultHeight, defaultWidth, defaultResolution);
		mapImage = null;
		mapPublisher.publishMap(defaultHeight, defaultWidth, (float)defaultResolution);
	}

	
	/**
	 * This method returns the instance of the only existing
	 * InteractiveMapController instance.
	 * @return An instance of InteractiveMapController
	 */
	public static synchronized InteractiveMapController getInstance(){
		if (instance == null){
			instance = new InteractiveMapController();
		}
		
		return instance;
	}
	
	/**
	 * Accepts a map view to be utilized by this controller. All actions made by this controller
	 * will use this map view.
	 * @param navMapView The map view to be used by this controller.
	 */
	public void setNavMapView(InteractiveMapViewLayered navMapView){
		this.navMapView = navMapView;
		navMapView.createGrid(navMapModel.getHeight(), navMapModel.getWidth(), (int)navMapModel.getCellSize());
		navMapView.setBackgroundImage(mapImage);
	}
	/**
	 * Returns the map model being used by the controller.
	 * @return The map model being used by the controller.
	 */
	public NavigationMapModel getMapModel(){
		return navMapModel;
	}
	
	/**
	 * Removes the highlighted state from all grid cells
	 */
	public void clearCells(){
		for (JLabel cell : highlightedCells){
			cell.setBackground(Color.white);
			//cell.setOpaque(false);
		}
		highlightedCells.clear();
		navMapModel.clearAllCells();
	}
	
	/**
	 * Highlights the given cell. If the clearSelection flag is true,
	 * all previously highlighted cells will be un-highlighted.
	 * @param target The JLabel cell to be highlighted.
	 * @param clearSelection If set true, all previously highlighted cells
	 * 							will be un-highlighted.
	 */
	public void highlightCell(JLabel target, boolean clearSelection){
		//Clear previously highlighted cells?
		if (clearSelection) {
			//Clear the highlighted cells
			for (JLabel cell : highlightedCells){
				cell.setBackground(Color.white);
				//cell.setOpaque(false);
			}
			navMapModel.clearAllCells();
			//Was the target cell not highlighted previously?
			if (!highlightedCells.contains(target)){
				//If not: Highlight it
				target.setBackground(Colors.lblSelectColor);
				//target.setOpaque(true);
				highlightedCells.clear();
				highlightedCells.add(target);
				String cellTxt = target.getText();
				navMapModel.toggleSelectCell(Integer.parseInt(cellTxt.split(",")[1]), 
						Integer.parseInt(cellTxt.split(",")[0]));
			} else{
				//If it was highlighted, clear it.
				highlightedCells.clear();
				String cellTxt = target.getText();
				navMapModel.toggleSelectCell(Integer.parseInt(cellTxt.split(",")[1]), 
						Integer.parseInt(cellTxt.split(",")[0]));
			}		
		}else{
			//Do not clear previously highlighted cells
			String cellTxt = target.getText();
			navMapModel.toggleSelectCell(Integer.parseInt(cellTxt.split(",")[1]), 
					Integer.parseInt(cellTxt.split(",")[0]));
			
			if (!highlightedCells.contains(target)){
				target.setBackground(Colors.lblSelectColor);
				highlightedCells.add(target);
				//target.setOpaque(true);
			}
			else{
				target.setBackground(Color.white);
				highlightedCells.remove(target);
				//target.setOpaque(false);
			}
		}
	}
	
	/**
	 * Alters the current map model to reflect new data.
	 * @param data The new data to be applied to the map model
	 */
	public void getNewMapData(int[] data){
		navMapModel.getNewMapData(data);
	}
	
	/**
	 * Updates the location on the map view of the named robot to the location given by the Pair parameter
	 * @param robotName The robot whose location needs to be updated
	 * @param location The new location of the robot.
	 */
	public void updateRobotLocation(String robotName, Pair<Integer, Integer> location){
		navMapView.updateRobotLocation(robotName, location);
	}
	
	
	//Mouse listeners
	Component lastDrag = new JLabel();
	
	/**
	 * This method is called whenever the mouse is dragged. It highlights
	 * any cell the mouse is dragged over.
	 */
	@Override
	public void mouseDragged(MouseEvent e) {
		//If this is the first mouseDragged call since
		//the last time the mouse was released, and if 
		//the control button is not held down, clear
		//the currently highlighted cells.		
		if (!isDragging && !e.isControlDown()){
			clearCells();
		}
		isDragging = true;
	
		//Highlight the cell the mouse is currently over.
		Component comp = e.getComponent().getComponentAt(e.getPoint());
		if (comp instanceof JLabel && comp != lastDrag){
			lastDrag = comp;
			highlightCell((JLabel)comp, false);
		}
	}


	@Override
	public void mouseMoved(MouseEvent e) {	}

	/**
	 * This method is called whenever the mouse is clicked. It toggles the highlighted status of
	 * a cell in the map view.
	 */
	@Override
	public void mouseClicked(MouseEvent e) {
		//Highlight the clicked cell. If control is down, do not
		//deselect already highlighted cells.
		JLabel target = (JLabel)e.getComponent().getComponentAt(e.getPoint());
		highlightCell(target, (e.isControlDown() ? false : true));
	}


	@Override
	public void mouseEntered(MouseEvent arg0) {}


	@Override
	public void mouseExited(MouseEvent arg0) {}


	@Override
	public void mousePressed(MouseEvent arg0) {}


	@Override
	public void mouseReleased(MouseEvent arg0) {
		isDragging = false;
	}
}
