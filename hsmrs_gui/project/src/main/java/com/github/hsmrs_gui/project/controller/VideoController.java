/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	VideoController.java											***
***		This class defines the controller which facilitates the		***
***		display of video data through available views. This class	***
****	follows the Singleton design pattern.						***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.controller;

import java.awt.image.BufferedImage;

import com.github.hsmrs_gui.project.GuiNode;

import src.main.java.com.github.hsmrs_gui.project.view.situational_awareness.ImageView;

public class VideoController {

	private static VideoController instance;
	private String source;
	private ImageView imgView;
	
	private VideoController(){}
	
	/**
	 * Returns the instance of the only existing VideoController instance.
	 * @return The instance of the only existing VideoController instance.
	 */
	public static VideoController getInstance(){
		if (instance == null){
			instance = new VideoController();
		}
		return instance;
	}
	
	/**
	 * Sets the current image view through which to display video data.
	 * @param imgView The ImageView used to display video data.
	 */
	public void setImageView(ImageView imgView){
		this.imgView = imgView;
	}
	
	/**
	 * Receive a new image to be processed and passed to the set ImageView to be
	 * displayed. If the source is not the desired source or if there is no set
	 * ImageView, the image passed in this call will be lost.
	 * @param source The source of this image as a String/
	 * @param img	A BufferedImage for the passed image.
	 */
	public synchronized void receiveNewImage(String source, BufferedImage img){
		//TODO Image muxing should happen here.
		if (imgView != null){
		imgView.setImage(img);
		}
	}
}
