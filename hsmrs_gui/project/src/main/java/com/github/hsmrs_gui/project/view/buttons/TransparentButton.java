/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	TransparentButton.java											***
***		This class defines a transparent button. It retains all of	***
***		the functionality of a JButton while extending it to react	***
***		to mouse events.											***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.view.buttons;

import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.ImageIcon;
import javax.swing.JButton;

public class TransparentButton extends JButton implements MouseListener {

	/**
	 * The constructor for the TransparentButton class.
	 * @param string The String to be displayed on the button.
	 */
	public TransparentButton(String string) {
		super(string);
		init();
	}

	/**
	 * The constructor for the TransparentButton class.
	 * @param imageIcon The image to display on the button.
	 */
	public TransparentButton(ImageIcon imageIcon) {
		super(imageIcon);
		init();
	}

	public TransparentButton(String string, ImageIcon imageIcon) {
		super(string, imageIcon);
		init();
	}

	private void init() {
		this.setOpaque(false);
		this.setContentAreaFilled(false);
		this.setBorderPainted(false);
		this.setFocusable(false);
		
		this.addMouseListener(this);
	}
	
	public void reset() {
		this.setOpaque(false);
		this.setContentAreaFilled(false);
		this.setBorderPainted(false);
	}

	@Override
	public void mouseClicked(MouseEvent e) {}

	@Override
	public void mouseEntered(MouseEvent e) {
		this.setOpaque(true);
		this.setContentAreaFilled(true);
		this.setBorderPainted(true);
	}

	@Override
	public void mouseExited(MouseEvent e) {
		this.setOpaque(false);
		this.setContentAreaFilled(false);
		this.setBorderPainted(false);
	}

	@Override
	public void mousePressed(MouseEvent e) {}

	@Override
	public void mouseReleased(MouseEvent e) {}
}
