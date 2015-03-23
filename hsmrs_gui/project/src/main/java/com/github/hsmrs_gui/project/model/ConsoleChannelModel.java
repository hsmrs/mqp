/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	ConsoleChannelModel.java										***
***		This class models a channel which exists within a console.	***
***		Console channels have names and a String which makes up		***
***		their logs. Logs are encoded as HTML.						***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model;

public class ConsoleChannelModel {

	private String name;
	private String log;
	
	/**
	 * Constructor for the ConsoleChannelModel class. All channels
	 * must have names.
	 * @param name The name of this ConsoleChannelModel.
	 */
	public ConsoleChannelModel(String name){
		this.name = name;
		log = "";
	}
	
	/**
	 * Gets the name of this ConsoleChannelModel.
	 * @return The name of this ConsoleChannelModel.
	 */
	public String getName(){
		return name;
	}
	
	/**
	 * Adds a log entry to this console channel.
	 * @param logEntry The log entry to be added to the channel. 
	 */
	public void addLogEntry(String logEntry){
		log += logEntry + "<br>";
	}
	
	/**
	 * Gets the contents of this console channel's log as a formatted HTML string.
	 * @return The contents of this console channel's log as HTML
	 */
	public String getLog(){
		return "<html>" + log + "</html>";
	}
	
	/**
	 * Deletes all of the text in the log.
	 */
	public void clearLog(){
		log = "";
	}
}
