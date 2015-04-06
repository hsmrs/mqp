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
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecificationListModel;
import src.main.java.com.github.hsmrs_gui.project.ros.ImageListener;
import src.main.java.com.github.hsmrs_gui.project.ros.NewTaskPublisher;
import src.main.java.com.github.hsmrs_gui.project.ros.RobotRegistrationListener;
import src.main.java.com.github.hsmrs_gui.project.ros.RolePublisher;
import src.main.java.com.github.hsmrs_gui.project.ros.SystemLogListener;
import src.main.java.com.github.hsmrs_gui.project.ros.UpdatedTaskListener;
import src.main.java.com.github.hsmrs_gui.project.ros.UpdatedTaskPublisher;
import src.main.java.com.github.hsmrs_gui.project.util.Pair;
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
	private MessageListener<hsmrs_framework.TaskMsg> updatedTaskListener;
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
	  System.out.println("Launching HSMRS GUI");
	  this.connectedNode = connectedNode;
    log = connectedNode.getLog();

    EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
            	RobotListModel rlm = RobotListModel.getInstance();
            	/*List<RobotModel> sampleRobots = new ArrayList<RobotModel>();
            	sampleRobots.add(new RobotModel("Hermes", new TaskModel()));
            	sampleRobots.add(new RobotModel("Oryx", new TaskModel()));
            	sampleRobots.add(new RobotModel("Husky", new TaskModel()));
            	sampleRobots.add(new RobotModel("Aero", new TaskModel()));
            	
            	sampleRobots.get(0).setLocation(new Pair<Integer, Integer>(1, 1));
            	sampleRobots.get(1).setLocation(new Pair<Integer, Integer>(2, 2));
            	sampleRobots.get(2).setLocation(new Pair<Integer, Integer>(3, 3));
            	sampleRobots.get(3).setLocation(new Pair<Integer, Integer>(4, 4));

            	
            	rlm.addRobot(sampleRobots.get(0));
            	rlm.addRobot(sampleRobots.get(1));
            	rlm.addRobot(sampleRobots.get(2));
            	rlm.addRobot(sampleRobots.get(3));
            	
            	TaskSpecificationListModel tslm = TaskSpecificationListModel.getInstance();
            	
            	TaskListModel tlm = TaskListModel.getInstance();
            	tlm.addTask(new TaskModel());
            	tlm.addTask(new TaskModel("Go to (x, y)", null));
            	tlm.addTask(new TaskModel("Search for sample", null, sampleRobots, "In progress"));
            	
            	*/
            	TaskListModel tlm = TaskListModel.getInstance();
            	TaskSpecificationListModel tslm = TaskSpecificationListModel.getInstance();
            	
            	List<String> goToParams = new ArrayList<String>();
            	goToParams.add("Location(x):Double");
            	goToParams.add("Location(y):Double");
            	TaskSpecification goToSpec = new TaskSpecification("GoTo", goToParams);
            	
            	List<String> followTagParams = new ArrayList<String>();
            	followTagParams.add("Tag ID:Integer");
            	TaskSpecification followTagSpec = new TaskSpecification("FollowTag", followTagParams);
            	
            	List<String> searchParams = new ArrayList<String>();
            	searchParams.add("Tag ID:Integer");
            	searchParams.add("Vertices:String");
            	TaskSpecification searchSpec = new TaskSpecification("Search", searchParams);
            	
            	tslm.addTaskSpecification(goToSpec);
            	tslm.addTaskSpecification(followTagSpec);
            	tslm.addTaskSpecification(searchSpec);
            	
            	ConsoleController.getInstance().createConsole(rlm.getRobotNames());
            	
            	
            	robotRegistrationListener = new RobotRegistrationListener(connectedNode);
            	systemLogListener = new SystemLogListener(connectedNode);
            	imageListener = new ImageListener(connectedNode);
            	updatedTaskListener = new UpdatedTaskListener();
            	UpdatedTaskPublisher.getInstance();
            	RolePublisher.getInstance();
            	
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
