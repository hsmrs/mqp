package src.main.java.com.github.hsmrs_gui.project.util;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.AbstractAction;

import src.main.java.com.github.hsmrs_gui.project.controller.TeleOpController;

public class TeleOpAction extends AbstractAction implements ActionListener{

	private String direction;
	
	public TeleOpAction(String name, String direction){
		super(name);
		
		this.direction = direction;
	}
	@Override
	public void actionPerformed(ActionEvent arg0) {
		TeleOpController.getInstance().handleTeleOpCommand(direction);
	}
}
