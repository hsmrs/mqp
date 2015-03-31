package src.main.java.com.github.hsmrs_gui.project.view.situational_awareness;

import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JPanel;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;
import net.miginfocom.swing.MigLayout;


public class SAPanel extends JPanel{

	private boolean showCamera = true;
	private boolean showMap = false;
	private ImageView imgView;
	private InteractiveMapViewLayered mapView;
	private MuxPanel muxPanel;
	
	/**
	 * The constructor for the SAPanel class. This class contains the subpanels which give the user situational awareness.
	 */
	public SAPanel(){
		setLayout(new MigLayout("insets 0, fill", "[fill]", "[fill]0[fill]"));
		imgView = new ImageView();
		mapView = new InteractiveMapViewLayered();
		muxPanel = new MuxPanel(this);
		
		add(muxPanel, "wrap");
		//add(imgView, "span 2 2, push, grow");
		add(imgView, "push, grow");
		//add(mapView, "push, grow");
		//add(new JButton("Hello World!"));
	}
	
	/**
	 * Sets which Situational Awareness view is currently on display.
	 * @param view Which view should be displayed: [imgView, mapView].
	 */
	public void setView(String view){
		if (view.equals("imgView")){
			remove(mapView);
			add(imgView, "push, grow");
		} else if (view.equals("mapView")){
			remove(imgView);
			add(mapView, "push, grow");
		}
		this.validate();
		repaint();
	}
}
