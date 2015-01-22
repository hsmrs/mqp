/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	ConsoleController.java											***
***		This class defines the controller which provides the outward***
***		facing capabilities of the Console module. It also 			***
***		facilitates communication between the view and model of the ***
***		console module.	This controller follows the Singleton		***		
***		design pattern - Only one console can exist at a time		***
**********************************************************************/


package src.main.java.com.github.hsmrs_gui.project.controller;

import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.model.ConsoleModel;
import src.main.java.com.github.hsmrs_gui.project.view.feedback.ConsoleView;

public class ConsoleController {

	private static ConsoleController instance;
	private ConsoleModel consoleModel;
	private ConsoleView consoleView;
	
	private ConsoleController(){}
	
	/**
	 * Retrieves the instance of the Singleton ConsoleController
	 * object.
	 * @return The instance of the unique ConsoleController.
	 */
	public static synchronized ConsoleController getInstance(){
		if (instance == null){
			instance = new ConsoleController();
		}
		return instance;
	}
	
	/**
	 * Sets the model of the console this controller will use.
	 * @param consoleModel The console's model.
	 */
	public void setConsoleModel(ConsoleModel consoleModel){
		this.consoleModel = consoleModel;
	}
	
	/**
	 * Creates a new console with a new console model using 
	 * a list of channel names
	 * @param channelNames The names of the channels to be
	 * 						included in the new console.
	 */
	public void createConsole(List<String> channelNames){
		this.consoleModel = new ConsoleModel(channelNames);
	}
	
	/**
	 * Sets the view of this console controller
	 * @param consoleView
	 */
	public void setConsoleView(ConsoleView consoleView){
		this.consoleView = consoleView;
	}
	
	/**
	 * Adds a channel to the console. This should be the only
	 * function which is called whenever a new channel is
	 * to be added to the console. Additions are reflected in the
	 * view and in the model.
	 * @param name The name of the channel to add.
	 */
	public void addConsoleChannel(String name){
		System.out.println("Add Console Channel in Console Controller");
		consoleModel.addChannel(name);
		consoleView.addChannel(name);
	}
	
	/**
	 * Removes a channel from the console. This should be the only
	 * function which is called whenever a channel is
	 * to be removed from the console. Removals are reflected in the
	 * view and in the model.
	 * @param name The name of the channel to remove.
	 */
	public void removeConsoleChannel(String name){
		consoleModel.removeChannel(name);
		consoleView.removeChannel(name);
	}
	
	/**
	 * Adds a log to the console to be displayed in the proper channels
	 * with the appropriate metadata.
	 * @param sender The sender of the log, used to determine the channel for the log
	 * @param timestamp When the log was submitted.
	 * @param text	The data of the log.
	 */
	public void addLog(String sender, long timestamp, String text){
		consoleModel.addLog(sender, timestamp, text);
		consoleView.setChannelLogText("All", consoleModel.getLogFor("All"));
		consoleView.setChannelLogText(sender, consoleModel.getLogFor(sender));
	}

	/**
	 * Returns the list of channel names.
	 * @return The list of channel names in this console.
	 */
	public List<String> getChannelNames() {
		return consoleModel.getChannelNames();
	}
}
