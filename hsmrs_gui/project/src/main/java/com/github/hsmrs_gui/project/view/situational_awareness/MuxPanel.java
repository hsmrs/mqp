package src.main.java.com.github.hsmrs_gui.project.view.situational_awareness;

import java.awt.FlowLayout;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.List;

import javax.swing.ButtonGroup;
import javax.swing.JPanel;
import javax.swing.JRadioButton;

import src.main.java.com.github.hsmrs_gui.project.util.Colors;

public class MuxPanel extends JPanel{
	
	public MuxPanel(final SAPanel parent){
		setLayout(new FlowLayout(FlowLayout.CENTER, 0, 0));
		
		ButtonGroup group = new ButtonGroup();
		setBackground(Colors.bannerColor);
		setSize(getWidth(), 30);
		
		JRadioButton cameraBtn = new JRadioButton("Camera");
		cameraBtn.setSelected(true);
		cameraBtn.addItemListener(new ItemListener() {
	         public void itemStateChanged(ItemEvent e) {         
	            if (e.getStateChange() == 1)
	            	parent.setView("imgView");
	         }           
	      });
		cameraBtn.setBackground(Colors.bannerColor);
		group.add(cameraBtn);
		add(cameraBtn);
		
		JRadioButton mapBtn = new JRadioButton("Map");
		mapBtn.addItemListener(new ItemListener() {
	         public void itemStateChanged(ItemEvent e) {    
	        	 if (e.getStateChange() == 1)
	        		 parent.setView("mapView");
	         }           
	      });
		mapBtn.setBackground(Colors.bannerColor);
		group.add(mapBtn);
		add(mapBtn);
		
	}

}
