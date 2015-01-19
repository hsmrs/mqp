package src.main.java.com.github.hsmrs_gui.project.view.situational_awareness;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JLayeredPane;
import javax.swing.JPanel;
import javax.swing.border.LineBorder;

import src.main.java.com.github.hsmrs_gui.project.controller.InteractiveMapController;
import src.main.java.com.github.hsmrs_gui.project.model.NavigationMapModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;
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
		
		createGrid(10, 10, 10);
		
		for (String robotName : RobotListModel.getInstance().getRobotNames()){
			RobotModel robot = RobotListModel.getInstance().getRobotModelByName(robotName);
			updateRobotLocation(robotName, robot.getLocation());
		}
		
		InteractiveMapController.getInstance().setNavMapView(this);
	}
	
	public void createGrid(int height, int width, int gridCellDimPixels){
		removeAll();
		
		labelMatrix = new JLabel[height][width];
		for (int y = 0; y < height; y++){
			for (int x = 0; x < width; x++){
				JLabel lbl = new JLabel();
				lbl.setBackground(Color.white);
				lbl.setOpaque(false);
				lbl.setText(x + "," + y);
				lbl.setMinimumSize(new Dimension(gridCellDimPixels, gridCellDimPixels));
				lbl.setPreferredSize(new Dimension(gridCellDimPixels, gridCellDimPixels));
				lbl.setMaximumSize(new Dimension(gridCellDimPixels, gridCellDimPixels));
				labelMatrix[y][x] = lbl;
				if (x == width - 1)
					add(lbl, "gap top 0, wrap");
				else
					add(lbl);
			}
		}
		
		for (String robotName : RobotListModel.getInstance().getRobotNames()){
			RobotModel robot = RobotListModel.getInstance().getRobotModelByName(robotName);
			updateRobotLocation(robotName, robot.getLocation());
		}
	}
	
	public void updateRobotLocation(String robotName, Pair<Integer, Integer> location){
		System.out.println("Update Robot Location for: " + robotName + " at: " + location.toString());
		
		RobotModel robot = RobotListModel.getInstance().getRobotModelByName(robotName);
		JLabel target = labelMatrix[location.Y][location.X];
		target.setOpaque(true);
		target.setBackground(robot.getColor());
		
		
		Pair<Integer, Integer> oldLocation = robot.getLocation();
		if (oldLocation.X != location.X && oldLocation.Y != location.Y){
			JLabel oldLabel = labelMatrix[oldLocation.Y][oldLocation.X];
			oldLabel.setOpaque(false);
			oldLabel.setBackground(Color.white);
		}
		
		robot.setLocation(location);
		
		revalidate();
		repaint();
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
