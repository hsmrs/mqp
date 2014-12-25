/**********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***	FeedbackPane.java												***
***		The feedback pane is a placeholder to allow quick switching	*** 
***		between different implementations of user feedback modules.	***
**********************************************************************/

package src.main.java.com.github.hsmrs_gui.project.view.feedback;

import java.awt.Color;

import javax.swing.JPanel;

import net.miginfocom.swing.MigLayout;

public class FeedbackPane extends JPanel{

	private ConsoleView consoleView;
	
	public FeedbackPane(){
		consoleView = new ConsoleView();
		
		setLayout(new MigLayout("insets 0, fill", "[][]", "[]"));
		
		add(consoleView, "left, span, grow");
	}
}
