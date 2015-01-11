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

package com.github.hsmrs_gui.project;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import src.main.java.com.github.hsmrs_gui.project.controller.ConsoleController;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.Task;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecificationListModel;
import src.main.java.com.github.hsmrs_gui.project.ros.ImageListener;
import src.main.java.com.github.hsmrs_gui.project.ros.NewTaskPublisher;
import src.main.java.com.github.hsmrs_gui.project.ros.RobotRegistrationListener;
import src.main.java.com.github.hsmrs_gui.project.ros.SystemLogListener;
import src.main.java.com.github.hsmrs_gui.project.view.MainFrame;

import java.awt.EventQueue;
import java.util.ArrayList;
import java.util.List;

public class GuiNode extends AbstractNodeMain {

	private static Log log;
	private static ConnectedNode connectedNode;
	private MessageListener<std_msgs.String> robotRegistrationListener;
	private MessageListener<std_msgs.String> systemLogListener;
	private MessageListener<sensor_msgs.Image> imageListener;
	private NewTaskPublisher taskPublisher;
	
	
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/gui_node");
  }
  
  public static Log getLog(){
	  return log;
  }
  
  public static ConnectedNode getConnectedNode(){
	  return connectedNode;
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
	  this.connectedNode = connectedNode;
    log = connectedNode.getLog();

    EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
            	RobotListModel rlm = RobotListModel.getRobotListModel();
            	List<RobotModel> sampleRobots = new ArrayList<RobotModel>();
            	sampleRobots.add(new RobotModel("Hermes", new Task()));
            	sampleRobots.add(new RobotModel("Oryx", new Task()));
            	sampleRobots.add(new RobotModel("Husky", new Task()));
            	sampleRobots.add(new RobotModel("Aero", new Task()));
            	
            	rlm.addRobot(sampleRobots.get(0));
            	rlm.addRobot(sampleRobots.get(1));
            	rlm.addRobot(sampleRobots.get(2));
            	rlm.addRobot(sampleRobots.get(3));
            	
            	TaskSpecificationListModel tslm = TaskSpecificationListModel.getInstance();
            	
            	TaskListModel tlm = TaskListModel.getInstance();
            	tlm.addTask(new Task());
            	tlm.addTask(new Task("Go to (x, y)", null));
            	tlm.addTask(new Task("Search for sample", null, sampleRobots, "In progress"));
            	
            	ConsoleController.getInstance().createConsole(rlm.getRobotNames());
            	
            	
            	robotRegistrationListener = new RobotRegistrationListener(connectedNode);
            	systemLogListener = new SystemLogListener(connectedNode);
            	imageListener = new ImageListener(connectedNode);
            	
            	taskPublisher = new NewTaskPublisher(connectedNode);
            	
            	MainFrame gui = new MainFrame(rlm, tlm);
            	gui.setVisible(true);	
            }});

    Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("display", std_msgs.String._TYPE);
    subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
      @Override
      public void onNewMessage(std_msgs.String message) {
        log.info("I heard: \"" + message.getData() + "\"");
      }
    });
  }
}
