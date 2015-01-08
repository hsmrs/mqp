package src.main.java.com.github.hsmrs_gui.project.view.situational_awareness;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.image.BufferedImage;
import java.io.File;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JLayeredPane;
import javax.swing.JPanel;
import javax.swing.border.LineBorder;

import src.main.java.com.github.hsmrs_gui.project.controller.InteractiveMapController;
import src.main.java.com.github.hsmrs_gui.project.model.NavigationMapModel;
import net.miginfocom.swing.MigLayout;

public class InteractiveMapViewLayered extends JPanel{

	private JLabel[][] labelMatrix;
	private int defaultHeight = 20;
	private int defaultWidth = 20;
	private int x, y;
	private int height;
	private int width;
	private int gridCellDim;
	
	private BufferedImage mapImage;
	
	public InteractiveMapViewLayered() {
		
		setLayout(new MigLayout("insets 0, gap 0, align center"));
		addMouseListener(InteractiveMapController.getInstance());
		addMouseMotionListener(InteractiveMapController.getInstance());
		
		createGrid(1000, 1000, 10);
		InteractiveMapController.getInstance().setNavMapView(this);
	}
	
	public void createGrid(int height, int width, int gridCellDim){
		removeAll();
		
		labelMatrix = new JLabel[height][width];
		for (int y = 0; y < height; y++){
			for (int x = 0; x < width; x++){
				JLabel lbl = new JLabel();
				lbl.setBackground(Color.white);
				//lbl.setBorder(new LineBorder(Color.black));
				lbl.setOpaque(false);
				lbl.setMinimumSize(new Dimension(gridCellDim, gridCellDim));
				lbl.setPreferredSize(new Dimension(gridCellDim, gridCellDim));
				lbl.setMaximumSize(new Dimension(gridCellDim, gridCellDim));
				labelMatrix[y][x] = lbl;
				if (x == width - 1)
					add(lbl, "gap top 0, wrap");
					//jlp.add(lbl, 1);
				else
					add(lbl);
			}
		}
	}
	
	public void setBackgroundImage(BufferedImage backgroundImage){
		mapImage = backgroundImage;
	}
	
	@Override
	 public void paintComponent(Graphics g) {
	    super.paintComponent(g);

	    // Draw the background image.
	    g.drawImage(mapImage, 0, 0, this);
	  }
}
