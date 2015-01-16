package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.awt.Font;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.ListModel;
import javax.swing.border.EmptyBorder;

import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import net.miginfocom.swing.MigLayout;

public class TaskListPanel extends JPanel{
	
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
		btnAddTask = new JButton("New task");
		btnAddTask.addActionListener(TaskController.getInstance());
		btnAddTask.setActionCommand("New task");
		btnRemoveTask = new JButton("Remove task");
		btnRemoveTask.addActionListener(TaskController.getInstance());
		btnRemoveTask.setActionCommand("Remove task");
		
		this.setBackground(Color.white);
		this.setBorder(BorderFactory.createEmptyBorder());
		this.setLayout(new MigLayout("insets 0", "[left, fill]0[]", "[]0[]0[fill]"));
		
		this.add(lblTitle, "span, wrap");
		this.add(btnAddTask, "gapleft 0%, gapright 0%");
		this.add(btnRemoveTask, "gapleft 0%, gapright 0%, wrap");
		this.add(taskListView, "span, push, grow, wrap");
		
	}
	
	public List<TaskModel> getSelectedTask(){
		return taskListView.getSelectedItems();
	}

}
