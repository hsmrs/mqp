package src.main.java.com.github.hsmrs_gui.project.controller;



import java.util.List;

import javax.swing.AbstractAction;

import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;

public class TeleOpController  {

	private static TeleOpController instance;
	
	public static TeleOpController getInstance(){
		if (instance == null){
			instance = new TeleOpController();
		}
		
		return instance;
	}

	public void handleTeleOpCommand(String direction) {
		List<RobotModel> robots = RobotListModel.getInstance().getRobots();
		for (RobotModel robot : robots){
			if (robot.getStatus().equals("Tele-Op")){
				robot.sendTeleOpCommand(direction);
			}
		}
	}
	


}
