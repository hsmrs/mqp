package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.awt.MenuItem;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.border.MatteBorder;

import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.RenderableComponent;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.view.buttons.TransparentButton;
import src.main.java.com.github.hsmrs_gui.project.view.list.*;
import net.miginfocom.swing.MigLayout;

public class TaskListComponent extends RenderableComponent<TaskModel> {

	TaskModel task;
	JPanel panel;
	JLabel lblName;
	JLabel lblOwners;
	JLabel lblStatus;
	JPopupMenu popupMenu;

	public TaskListComponent() {
	}

	/**
	 * Create a JComponent from a ListItem parameterized as a TaskModel.
	 * @param The ListItem to render a JComponent from.
	 * @return A JComponent rendered from the given ListItem.
	 */
	@Override
	public JComponent create(ListItem<TaskModel> listItem) {
		task = listItem.getListObject();

		panel = new JPanel(new MigLayout("fill", "[]", "[]10[]10[]"));
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor
				: Color.WHITE);
		panel.setBorder(new MatteBorder(0, 0, 1, 0, Color.GRAY));

		lblName = new JLabel(task.getType());
		lblName.setOpaque(false);

		lblStatus = new JLabel("Status: " + task.getStatus());
		lblName.setOpaque(false);

		String stringOfOwners = "Owners: ";
		List<RobotModel> owners = task.getOwners();
		for (int i = 0; i < owners.size(); i++) {
			stringOfOwners += owners.get(i).getName()
					+ ((i == (owners.size() - 1)) ? ("") : (", "));
		}
		lblOwners = new JLabel(stringOfOwners);
		lblOwners.setOpaque(false);

		// compressButton = new TransparentButton(compress);
		// compressButton.setMargin(new Insets(0, 0, 0 ,0));
		// compressButton.setActionCommand("compress");
		// compressButton.addActionListener(new
		// ListItemExpandListener<Event>(listItem));

		// expandButton = new TransparentButton(expand);
		// expandButton.setMargin(new Insets(0, 0, 0 ,0));
		// expandButton.setActionCommand("expand");
		// expandButton.addActionListener(new
		// ListItemExpandListener<Event>(listItem));

		panel.add(lblName, "pushx, growx, wrap");
		panel.add(lblStatus, "pushx, growx, wrap");
		panel.add(lblOwners);
		// panel.add(expandButton, "alignx right");

		return panel;
	}

	/**
	 * Recreate a JComponent from a ListItem parameterized as a TaskModel.
	 * @param The ListItem to render a JComponent from.
	 */
	@Override
	public void update(ListItem<TaskModel> listItem) {
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor
				: Color.WHITE);
		
		lblStatus = new JLabel("Status: " + task.getStatus());
		
		String stringOfOwners = "Owners: ";
		List<RobotModel> owners = task.getOwners();
		for (int i = 0; i < owners.size(); i++) {
			stringOfOwners += owners.get(i).getName()
					+ ((i == (owners.size() - 1)) ? ("") : (", "));
		}
		lblOwners = new JLabel(stringOfOwners);
		lblOwners.setOpaque(false);

		panel.removeAll();

		panel.add(lblName, "pushx, growx, wrap");
		panel.add(lblStatus, "pushx, growx, wrap");
		panel.add(lblOwners);

		// if(!listItem.isExpanded()) {
		// panel.add(endDateLabel);
		// expandButton.reset();
		// panel.add(expandButton, "alignx right");
		// } else {
		// panel.add(endDateLabel, "span 2, wrap");
		// panel.add(isTeamLabel, "span 2, wrap");
		// if(hasDescription) panel.add(descriptionText,
		// "growx, span 2, wrap");
		// panel.add(createdByLabel);
		// compressButton.reset();
		// panel.add(compressButton, "alignx right");
		// }

		panel.repaint();

	}
}
