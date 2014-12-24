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
	
	public String getName(){
		return name;
	}
	
	public void addLogEntry(String logEntry){
		log += logEntry + "<br>";
	}
	
	public String getLog(){
		return "<html>" + log + "</html>";
	}
	
	public void clearLog(){
		log = "";
	}
}
