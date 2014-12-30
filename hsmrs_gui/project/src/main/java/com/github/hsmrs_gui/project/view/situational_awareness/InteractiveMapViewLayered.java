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
import net.miginfocom.swing.MigLayout;

public class InteractiveMapViewLayered extends JPanel{

	private JLabel[][] labelMatrix;
	private int defaultHeight = 20;
	private int defaultWidth = 20;
	private int x, y;
	private int height;
	private int width;
	private int gridCellDim;
	
	private String INSTITUTE_PARK_MAP_PATH = "";
	private final double INSTITUTE_PARK_MAP_SCALE = 6.3876889849; //pixels per meter
	private BufferedImage mapImage;
	
	public InteractiveMapViewLayered() {
		INSTITUTE_PARK_MAP_PATH = String.format("%s/project/%s", System.getProperty("user.dir"), this.getClass().getPackage().getName().replace(".", "/"));
		INSTITUTE_PARK_MAP_PATH += "/images/institute_park_edited.jpg";
		setLayout(new MigLayout("insets 0, gap 0, align center"));
		addMouseListener(InteractiveMapController.getInstance());
		addMouseMotionListener(InteractiveMapController.getInstance());
		
		mapImage = null;
		try{
			mapImage = ImageIO.read(new File(INSTITUTE_PARK_MAP_PATH));
			height = (int)Math.round((double)mapImage.getHeight() / INSTITUTE_PARK_MAP_SCALE) + 10; //The 10 is a fudge factor. I dont know why it is needed.
			width = (int)((double)mapImage.getWidth() / INSTITUTE_PARK_MAP_SCALE);
			gridCellDim = (int) Math.round(INSTITUTE_PARK_MAP_SCALE);
		}
		catch(Exception e){
			System.out.println("Error reading image for interactive map image");
			height = defaultHeight;
			width = defaultWidth;
			gridCellDim = 500/defaultWidth;
		}
		JLabel lblMapImage = new JLabel(new ImageIcon(mapImage));
		
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
//		for (int x = 0; x < width; x++){
//			for (int y = 0; y < height; y++){
//				JLabel lbl = new JLabel();
//				lbl.setBackground(Color.white);
//				lbl.setBorder(new LineBorder(Color.black));
//				lbl.setOpaque(false);
//				lbl.setPreferredSize(new Dimension(500/defaultWidth, 500/defaultHeight));
//				lbl.setMaximumSize(new Dimension(500/defaultWidth, 500/defaultHeight));
//				labelMatrix[y][x] = lbl;
//				if (y == height - 1)
//					add(lbl, "gap top 0, wrap");
//					//jlp.add(lbl, 1);
//				else
//					add(lbl);
//			}
//		}
		//add(jlp);
	}
	
	@Override
	 public void paintComponent(Graphics g) {
	    super.paintComponent(g);

	    // Draw the background image.
	    g.drawImage(mapImage, 0, 0, this);
	  }
}
