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
	
	/**
	 * The constructor for the ImageView class.
	 */
	public ImageView() {
		String path = String.format("%s/project/%s", System.getProperty("user.dir"), this.getClass().getPackage().getName().replace(".", "/"));
	    path += "/images/wall_e.jpg";
		VideoController.getInstance().setImageView(this);
		setBackground(Color.black);
		BufferedImage img = null;
		try{
		img = ImageIO.read(new File(path));
		}
		catch(Exception e){
			System.out.println("Error reading image in ImageView");
		}
		ImageIcon ic;
	    if (img != null) {
	        ic = new ImageIcon(img);
	        lblImage = new JLabel(ic);
	    } else {
	        System.out.println("Couldn't find file: " + path);
	        lblImage = new JLabel();
	    }		
	    currentImg = img;
	}
	
	/**
	 * Sets the image to be displayed by this panel.
	 * @param newImg The image to be displayed.
	 */
	public void setImage(BufferedImage newImg){
		currentImg = newImg;
		lblImage.setIcon(new ImageIcon(newImg));
		repaint();
	}
	
	/**
	 * Paints the image to the component with a relative scale.
	 */
	@Override
	protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        int w = currentImg.getWidth();
        int h = currentImg.getHeight();
        if (w >= h){
        	//System.out.println("Paint greater width");
        	double scale = (double)h / w;
        	//g.drawImage(currentImg, 0, 0, getWidth(), (int)(getWidth() * scale), this);
        	Image scaledCurrentImage = currentImg.getScaledInstance(getWidth(), -1, Image.SCALE_SMOOTH);
        	g.drawImage(scaledCurrentImage, 0, 0, this);
        }
        else{
        	//System.out.println("Paint greater height");
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
