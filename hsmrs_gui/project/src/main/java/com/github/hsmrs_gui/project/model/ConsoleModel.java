/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	ConsoleModel.java												***
***		This class models a console module. The console module is	***
***		responsible for displaying log messages and organizing them	***
***		by source. Each source's log messages are shown in its own	***
***		channel.													***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.model;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import src.main.java.com.github.hsmrs_gui.project.view.feedback.ConsoleView;

public class ConsoleModel {
	
	ArrayList<ConsoleChannelModel> channels;
	
	/**
	 * Default constructor for the ConsoleModel. This creates a console
	 * with only the findamental channels: "All" and "System".
	 */
	public ConsoleModel(){
		this.channels = new ArrayList<ConsoleChannelModel>();
		ConsoleChannelModel ccmAll = new ConsoleChannelModel("All");
		ConsoleChannelModel ccmSystem = new ConsoleChannelModel("System");
		channels.add(ccmAll);
		channels.add(ccmSystem);
	}
	
	/**
	 * Overloaded constructor for the ConsoleModel class. This version
	 * accepts a list of initial channels to create along with the console,
	 * @param channelNames The initial channel names of this console.
	 */
	public ConsoleModel(List<String> channelNames){
		this();
		for (String channelName : channelNames){
			addChannel(channelName);
		}
	}
	
	/**
	 * Adds a channel of the given name to this console.
	 * @param name The name of the new channel.
	 */
	public void addChannel(String name){
		this.channels.add(new ConsoleChannelModel(name));
	}
	
	/**
	 * Removes a channel of the given name from this console.
	 * @param name The name of the channel to be removed.
	 */
	public void removeChannel(String name){
		ConsoleChannelModel ccm = getChannel(name);
		channels.remove(ccm);
	}
	
	/**
	 * Add a log to the console. The log is displayed in the "All" channel and
	 * also in the channel whose name matches the sender's name. All logs are
	 * timestamped and these timestamps are shown to the user.
	 * @param sender The name of the sender of the log.
	 * @param timestamp The seconds from the epoch for when this log was written.
	 * @param text The text of the log.
	 */
	public void addLog(String sender, long timestamp, String text){
		ConsoleChannelModel ccmAll = getChannel("All");
		ConsoleChannelModel ccmSender = getChannel(sender);
		
		//Markup the log using HTML. Timestamp and sender are shown in bold.
		ccmAll.addLogEntry("<b>[" + String.valueOf(timestamp) + "] "
				+ sender + ": </b>" + text);
		
		ccmSender.addLogEntry("<b>[" + String.valueOf(timestamp) + "] </b>"
				+ text);
	}

	/**
	 * Returns the full log recorded by the channel of the given name.
	 * @param channelName The name of the channel whose log is being retreived.
	 * @return A String representing the HTML marked up log.
	 */
	public String getLogFor(String channelName) {
		return getChannel(channelName).getLog();
	}
	
	/**
	 * Gets the channel of the given name.
	 * @param channelName The name of the desired channel.
	 * @return The channel of this console with the given name.
	 */
	public ConsoleChannelModel getChannel(String channelName){
		for (ConsoleChannelModel ccm : channels){
			if (ccm.getName().equals(channelName)){
				return ccm;
			}
		}
		return null;
	}

	/**
	 * Returns a list of channel names belonging to this console.
	 * @return A list of channel names belonging to this console.
	 */
	public List<String> getChannelNames() {
		ArrayList<String> rtnList = new ArrayList<String>();
		for (ConsoleChannelModel ccm : channels){
			rtnList.add(ccm.getName());
		}
		return rtnList;
	}
}
