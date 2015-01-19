package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPopupMenu;

import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;

public class TaskListComponentPopup extends JPopupMenu implements ActionListener{
	
	private TaskModel task;
	private JComponent parent;

	public TaskListComponentPopup(TaskModel task, JComponent parent){
		this.task = task;
		this.parent = parent;
		
		JMenuItem itemAddOwner = new JMenuItem("Add Owner");
		itemAddOwner.addActionListener(this);
		JMenuItem itemRemoveOwner = new JMenuItem("Remove Owner");
		itemRemoveOwner.addActionListener(this);
		JMenuItem itemRemove = new JMenuItem("Remove");
		itemRemove.addActionListener(this);
		add(itemAddOwner);
		add(itemRemoveOwner);
		add(itemRemove);
	}
	
	public void actionPerformed(ActionEvent e) {
		JMenuItem item = (JMenuItem) e.getSource();

		if (item.getText().equals("Add Owner")) {
			List<String> possibilities = RobotListModel.getInstance()
					.getRobotNames();
			List<String> copy = new ArrayList<String>(possibilities);
			
			for (String name : copy){
				if (task.getOwners().contains(RobotListModel.getInstance().getRobotModelByName(name))){
					possibilities.remove(name);
				}
			}
			
			if (possibilities.size() == 0) return;
			
			String s = (String) JOptionPane.showInputDialog(parent,
					"Select a new owner for task: " + task.getName(),
					"Add owner", JOptionPane.PLAIN_MESSAGE, null,
					possibilities.toArray(), possibilities.get(0));

			if ((s != null) && (s.length() > 0)) {
				TaskController.getInstance().addOwnerToTask(task, s);
				return;
			}
		} else if (item.getText().equals("Remove Owner")) {
			if (task.getOwners().size() == 0) {

			} else {
				Object[] possibilities = new Object[task.getOwners().size()];
				for (int i = 0; i < possibilities.length; i++) {
					possibilities[i] = task.getOwners().get(i).getName();
				}
				String s = (String) JOptionPane
						.showInputDialog(
								parent,
								"Select an owner to remove for task: "
										+ task.getName(), "Remove owner",
								JOptionPane.PLAIN_MESSAGE, null, possibilities,
								possibilities[0]);

				if ((s != null) && (s.length() > 0)) {
					TaskController.getInstance().removeOwnerFromTask(task, s);
					return;
				}
			}
		} else if (item.getText().equals("Remove")) {
			TaskListModel.getInstance().removeTask(task);
			return;
		}
	}
	
}
