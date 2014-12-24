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
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JLabel;


public class InteractiveMapController implements MouseListener, MouseMotionListener{

	private static InteractiveMapController instance;
	private ArrayList<JLabel> highlightedCells;
	private ConsoleController consoleController;
	private boolean isDragging = false;
	
	/**
	 * Constructor for the InteractiveMapController class
	 */
	private InteractiveMapController(){
		consoleController = ConsoleController.getInstance();
		highlightedCells = new ArrayList<JLabel>();
	}

	
	/**
	 * This method returns the instance of the only existing
	 * InteractiveMapController instance.
	 * @return An instance of InteractiveMapController
	 */
	public static InteractiveMapController getInstance(){
		if (instance == null){
			instance = new InteractiveMapController();
		}
		
		return instance;
	}
	
	/**
	 * Removes the highlighted state from all grid cells
	 */
	public void clearCells(){
		for (JLabel cell : highlightedCells){
			cell.setBackground(Color.white);
		}
		highlightedCells.clear();
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
			}
			//Was the target cell not highlighted previously?
			if (!highlightedCells.contains(target)){
				//If not: Highlight it
				target.setBackground(Color.yellow);
				highlightedCells.clear();
				highlightedCells.add(target);
			} else{
				//If it was highlighted, clear it.
				highlightedCells.clear();
			}		
		}else{
			//Do not clear previously highlighted cells
			target.setBackground(Color.yellow);
			highlightedCells.add(target);
		}
	}

	//Mouse listeners

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
		if (comp instanceof JLabel){
			highlightCell((JLabel)comp, false);
		}
	}


	@Override
	public void mouseMoved(MouseEvent e) {	}


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
