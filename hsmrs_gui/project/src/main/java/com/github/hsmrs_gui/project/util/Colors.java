/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	Colors.java														***
***		This class is a container for all colors used throughout	***
***		the GUI and any methods that may pertain to colors.			***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.util;

import java.awt.Color;

public class Colors {

	public static final Color selectionColor = new Color(204, 232, 252);
	public static final Color lblSelectColor = new Color(255, 255, 0);
	public static final Color[] lblRobotPoseColors = {
		new Color(255, 0, 0), //Red
		new Color(0, 255, 0), //Green
		new Color(0, 0, 255), //Blue
		new Color(255, 128, 0), //Orange
		new Color(0, 255, 255), //Teal
		new Color(204, 0, 204), //Purple
		new Color(192, 192, 192) //Grey
	};
	private static int lblRobotPoseColorIndex = 0;
	public static final Color lblObstacle = new Color(0, 0, 0);
	public static final Color bannerColor = Color.decode("0XC0BCB6");
	
	public static Color chooseRobotColor(){
		Color returnColor = lblRobotPoseColors[lblRobotPoseColorIndex];
		lblRobotPoseColorIndex++;
		return returnColor;
	}
}
