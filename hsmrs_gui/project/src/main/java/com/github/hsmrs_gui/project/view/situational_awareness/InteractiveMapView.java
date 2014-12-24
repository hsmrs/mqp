package src.main.java.com.github.hsmrs_gui.project.view.situational_awareness;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.MouseMotionListener;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.Border;
import javax.swing.border.LineBorder;

import src.main.java.com.github.hsmrs_gui.project.controller.InteractiveMapController;
import net.miginfocom.swing.MigLayout;

public class InteractiveMapView extends JPanel
{

	private JLabel[][] labelMatrix;
	private int defaultHeight = 20;
	private int defaultWidth = 20;
	private int height;
	private int width;
	
	public InteractiveMapView() {
		setLayout(new MigLayout("insets 0, gap 0, align center"));
		setBackground(Color.black);
		addMouseListener(InteractiveMapController.getInstance());
		addMouseMotionListener(InteractiveMapController.getInstance());
		labelMatrix = new JLabel[defaultHeight][defaultWidth];
		for (int x = 0; x < defaultWidth; x++){
			for (int y = 0; y < defaultHeight; y++){
				JLabel lbl = new JLabel();
				lbl.setBackground(Color.white);
				lbl.setBorder(new LineBorder(Color.black));
				lbl.setOpaque(true);
				lbl.setPreferredSize(new Dimension(500/defaultWidth, 500/defaultHeight));
				lbl.setMaximumSize(new Dimension(500/defaultWidth, 500/defaultHeight));
				labelMatrix[y][x] = lbl;
				if (y == defaultHeight - 1)
					add(lbl, "gap top 0, wrap");
				else
					add(lbl);
			}
		}
	}
}
