package src.main.java.com.github.hsmrs_gui.project.view.situational_awareness;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;

import com.github.hsmrs_gui.project.GuiNode;

import src.main.java.com.github.hsmrs_gui.project.controller.VideoController;

public class ImageView extends JPanel{

	private BufferedImage currentImg;
	private JLabel lblImage;
	
	public ImageView() {
		System.out.println("----------------IMAGE VIEW---------------");
		String path = String.format("%s/project/%s", System.getProperty("user.dir"), this.getClass().getPackage().getName().replace(".", "/"));
	    path += "/images/wall_e.jpg";
		System.out.println(path);
		VideoController.getInstance().setImageView(this);
		setBackground(Color.black);
		//String paths = "/hsmrs_gui/project/src/main/resources/wall_e.jpg";//"/home/nick/mqp_ws/src/mqp/hsmrs_gui/project/src/main/java/com/github/hsmrs_gui/project/view/situational_awareness/images/wall_e.jpg";
		//java.net.URL imgURL = getClass().getResource(path);
		BufferedImage img = null;
		try{
		img = ImageIO.read(new File(path));
		}
		catch(Exception e){
			System.out.println("Meow");
		}
		ImageIcon ic;
	    if (img != null) {
	        ic = new ImageIcon(img);
	        lblImage = new JLabel(ic);
	        System.out.println("SUCCESS IMAGE ICON");
	    } else {
	        System.out.println("Couldn't find file: " + path);
	        lblImage = new JLabel();
	    }		
		//add(lblImage);
	    currentImg = img;
	}
	
	public void setImage(BufferedImage newImg){
		currentImg = newImg;
		lblImage.setIcon(new ImageIcon(newImg));
		repaint();
	}
	
	@Override
	protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        int w = currentImg.getWidth();
        int h = currentImg.getHeight();
        if (w >= h){
        	double scale = (double)h / w;
        	g.drawImage(currentImg, 0, 0, getWidth(), (int)(getWidth() * scale), this);
//        	Image scaledCurrentImage = currentImg.getScaledInstance(getWidth(), -1, Image.SCALE_SMOOTH);
//        	g.drawImage(scaledCurrentImage, 0, 0, -1, -1, this);
        }
        else{
        	double scale = (double)w / h;
        	int offset = (int)((getWidth() - (getHeight() * scale)) / 2);
        	g.drawImage(currentImg, offset, 0, (int)(getHeight() * scale), getHeight(), this);
//        	Image scaledCurrentImage = currentImg.getScaledInstance(-1, getHeight(), Image.SCALE_SMOOTH);
//        	g.drawImage(scaledCurrentImage, 0, 0, -1, -1, this);
        }
        //g.drawImage(currentImg, 0, 0, getWidth(), getHeight(), this);
        //g.drawImage(currentImg, 0, 0, 200, 200, this);
    }
}
