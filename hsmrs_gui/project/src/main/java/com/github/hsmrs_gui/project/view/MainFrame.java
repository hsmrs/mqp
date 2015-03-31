/*
 * Copyright (C) 2014 Nicholas Otero.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package src.main.java.com.github.hsmrs_gui.project.view;

import javax.swing.JFrame;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.EventQueue;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.InputMap;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JButton;
import javax.swing.ImageIcon;
import javax.swing.JTabbedPane;
import javax.swing.JPanel;
import javax.swing.KeyStroke;
import javax.swing.ListModel;

import src.main.java.com.github.hsmrs_gui.project.model.task.TaskParam;
import src.main.java.com.github.hsmrs_gui.project.ros.TeleOpPublisher;
import src.main.java.com.github.hsmrs_gui.project.util.TeleOpAction;
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotListView;
import src.main.java.com.github.hsmrs_gui.project.view.robot.RobotPanel;
import src.main.java.com.github.hsmrs_gui.project.view.situational_awareness.SAPanel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskListPanel;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskPanel;
import src.main.java.com.github.hsmrs_gui.project.view.feedback.FeedbackPane;
import net.miginfocom.swing.MigLayout;

import org.apache.commons.logging.Log;

import com.github.hsmrs_gui.project.GuiNode;

public class MainFrame extends JFrame {
	//private TaskListPanel taskPane;
	private JPanel rootPane;
	private TaskPanel taskPane;
	private RobotPanel robotPane;
	private FeedbackPane feedbackPane;
	private JTabbedPane bottomPane;
	private JTabbedPane centerPane;
	private JLabel taskList;
	private JLabel rightList;
	private JLabel console;
	private JLabel imageDisplay;
	private ImageIcon defaultImage;
	private ListModel robotListModel;
	private ListModel taskListModel;
	private SAPanel situAwareView;
	private Log log;

	/**
	 * This constructor creates the MainFrame class, which is the underlying GUI element which contains
	 * every part of the GUI.
	 * @param roboListModel The list model for robots used in this system.
	 * @param taskListModel The list model for tasks used in this system.
	 */
  	public MainFrame(ListModel roboListModel, ListModel taskListModel) {
  		this.robotListModel = roboListModel;
  		this.taskListModel = taskListModel;
  		this.log = GuiNode.getLog();
  		log.info("Initializing HSMRS GUI. Please wait.");
  		
       setTitle("HSMRS GUI");
       setExtendedState(getExtendedState()|JFrame.MAXIMIZED_BOTH );
       //setSize(700, 700);
       setLocationRelativeTo(null);
       setDefaultCloseOperation(EXIT_ON_CLOSE);        
       setup();
    }

  	/**
  	 * This method actually creates the GUI elements within the main frame.
  	 */
    public void setup(){
      Container cp = this.getContentPane(); 
      //cp.setLayout(new MigLayout("insets 0", "[20%]0[60%]0[20%]", "[75%]0[25%]"));
      //cp.setLayout(new BorderLayout());
      //cp.setBackground(Color.white);
      
      rootPane = new JPanel();
      rootPane.setLayout(new MigLayout("insets 0", "[20%]0[60%]0[20%]", "[75%]0[25%]"));
      rootPane.setBackground(Color.white);

      //Task List
      //taskPane = new TaskListPanel(taskListModel);
      taskPane = TaskPanel.getInstance();
      //taskPane.setMaximumSize(new Dimension(500, 200));
      
      //Situational Awareness View
      situAwareView = new SAPanel();

      //Robot List
      robotPane = RobotPanel.getInstance();
      robotPane.setListModel(robotListModel);
      
      //Console
      feedbackPane = new FeedbackPane();
      
//      cp.add(taskPane, "left, grow");
//      cp.add(situAwareView, "left, push, grow");
//      cp.add(robotPane, "right, push, grow, wrap");
//      cp.add(feedbackPane, "push, grow, span 3"); 
      
      rootPane.add(taskPane, "left, grow");
      rootPane.add(situAwareView, "left, push, grow");
      rootPane.add(robotPane, "right, push, grow, wrap");
      rootPane.add(feedbackPane, "push, grow, span 3"); 
      
      addAction("UP", TeleOpPublisher.UP);
      addAction("DOWN", TeleOpPublisher.DOWN);
      addAction("LEFT", TeleOpPublisher.LEFT);
      addAction("RIGHT", TeleOpPublisher.RIGHT);
      
      cp.add(rootPane);
      
      pack();
    }

    /**
     * Binds tele-op actions to keystrokes on the keyboard.
     * @param name The name of the keystroke to be bound.
     * @param direction The tele-op direction to be bound to the keystroke. Directions are defined in TeleOpPublisher.
     * @return The TeleOpAction that was bound to the keystroke.
     */
    public TeleOpAction addAction(String name, String direction)
	{
    	TeleOpAction action = new TeleOpAction(name, direction);

		KeyStroke pressedKeyStroke = KeyStroke.getKeyStroke(name);
		InputMap inputMap = rootPane.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
		inputMap.put(pressedKeyStroke, name);
		rootPane.getActionMap().put(name, action);

		return action;
	}
    
    public void receiveText(String text){

    }
}
