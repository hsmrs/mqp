package src.main.java.com.github.hsmrs_gui.project.util;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.AbstractAction;

import src.main.java.com.github.hsmrs_gui.project.controller.TeleOpController;

public class TeleOpAction extends AbstractAction implements ActionListener{

	private String direction;
	
	/**
	 * The constructor for the TeleOpAction class.
	 * @param name The name of the action.
	 * @param direction The direction of Tele-Op associated with this action.
	 */
	public TeleOpAction(String name, String direction){
		super(name);
		
		this.direction = direction;
	}
	
	/**
	 * This method is called whenever this action is invoked.
	 * @param arg0 This is unused.
	 */
	@Override
	public void actionPerformed(ActionEvent arg0) {
		TeleOpController.getInstance().handleTeleOpCommand(direction);
	}
}
