package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.awt.Font;
import java.awt.Point;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.ListModel;
import javax.swing.border.EmptyBorder;

import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItemListener;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import net.miginfocom.swing.MigLayout;

public class TaskListPanel extends JPanel implements ListItemListener<TaskModel>{
	
	private JLabel lblTitle;
	private SRList<TaskModel> taskListView;
	private JButton btnAddTask;
	private JButton btnRemoveTask;
	
	public TaskListPanel(ListModel listModel) {
		lblTitle = new JLabel("Task List", JLabel.CENTER);
		lblTitle.setBackground(Color.decode("0XC0BCB6"));
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));
		
		taskListView = new SRList<TaskModel>(listModel, new TaskListItemRenderer());
		taskListView.setBorder(BorderFactory.createEmptyBorder());
		taskListView.addListItemListener(this);
		btnAddTask = new JButton("New task");
		btnAddTask.addActionListener(TaskController.getInstance());
		btnAddTask.setActionCommand("New task");
		btnRemoveTask = new JButton("Remove task");
		btnRemoveTask.addActionListener(TaskController.getInstance());
		btnRemoveTask.setActionCommand("Remove task");
		
		//this.setBackground(Color.white);
		this.setBorder(BorderFactory.createEmptyBorder());
		this.setLayout(new MigLayout("insets 0, fill", "[left, fill]0[]", "[]0[]0[fill]push"));
		
		this.add(lblTitle, "span, pushx, growx, wrap");
		this.add(btnAddTask, "gapleft 0%, gapright 0%, pushx, growx");
		this.add(btnRemoveTask, "gapleft 0%, gapright 0%, pushx, growx, wrap");
		this.add(taskListView, "span, push, grow, wrap");
		
	}
	
	public List<TaskModel> getSelectedTask(){
		return taskListView.getSelectedItems();
	}

	public void update(){
		taskListView.updateComponents();
	}

	public void itemsSelected(List<TaskModel> listObjects) {
		// TODO Auto-generated method stub
		
	}

	public void itemDoubleClicked(TaskModel listObject) {
		// TODO Auto-generated method stub
		
	}

	public void itemRightClicked(TaskModel listObject, Point p) {
		TaskListComponentPopup popup = new TaskListComponentPopup(listObject, this);
		popup.show(this, p.x, p.y);
	}

	public void itemFocused(TaskModel listObject) {
		// TODO Auto-generated method stub
		
	}
}
