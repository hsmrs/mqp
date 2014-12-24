package src.main.java.com.github.hsmrs_gui.project.view.situational_awareness;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;

import com.github.hsmrs_gui.project.GuiNode;

import src.main.java.com.github.hsmrs_gui.project.controller.VideoController;

public class ImageView extends JPanel{

	private BufferedImage currentImg;
	private JLabel lblImage;
	
	public ImageView() {
		VideoController.getInstance().setImageView(this);
		setBackground(Color.black);
		lblImage = new JLabel();
		add(lblImage);
	}
	
	public void setImage(BufferedImage newImg){
		currentImg = newImg;
		lblImage.setIcon(new ImageIcon(newImg));
		repaint();
	}
	
//	 @Override
//	 protected void paintComponent(Graphics g) {
//		 super.paintComponent(g);
//		 if(currentImg != null){
//			 g.drawImage(currentImg, 0, 0, null); 
//		 }
//	}
}
