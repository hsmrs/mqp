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

import src.main.java.com.github.hsmrs_gui.project.model.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.RenderableComponent;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.view.buttons.TransparentButton;
import src.main.java.com.github.hsmrs_gui.project.view.list.*;
import net.miginfocom.swing.MigLayout;

public class TaskListComponent extends RenderableComponent<TaskModel> implements
		ActionListener {

	TaskModel task;
	JPanel panel;
	JLabel lblName;
	JLabel lblOwners;
	JLabel lblStatus;
	JPopupMenu popupMenu;

	public TaskListComponent() {
	}

	@Override
	public JComponent create(ListItem<TaskModel> listItem) {
		task = listItem.getListObject();

		panel = new JPanel(new MigLayout("fill", "[]", "[]10[]10[]"));
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor
				: Color.WHITE);
		panel.setBorder(new MatteBorder(0, 0, 1, 0, Color.GRAY));

		lblName = new JLabel(task.getName());
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

		popupMenu = new JPopupMenu();
		JMenuItem itemAddOwner = new JMenuItem("Add Owner");
		itemAddOwner.addActionListener(this);
		JMenuItem itemRemoveOwner = new JMenuItem("Remove Owner");
		itemRemoveOwner.addActionListener(this);
		JMenuItem itemRemove = new JMenuItem("Remove");
		itemRemove.addActionListener(this);
		popupMenu.add(itemAddOwner);
		popupMenu.add(itemRemoveOwner);
		popupMenu.add(itemRemove);
		panel.addMouseListener(new PopupListener(popupMenu));

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

	@Override
	public void update(ListItem<TaskModel> listItem) {
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor
				: Color.WHITE);

		if (listItem.isDirty()) {
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

	public void actionPerformed(ActionEvent e) {
		JMenuItem item = (JMenuItem) e.getSource();

		if (item.getText().equals("Add Owner")) {
			List<String> possibilities = RobotListModel.getInstance()
					.getRobotNames();
			List<String> copy = new ArrayList<String>(possibilities);
			
			for (String name : copy){
				if (task.getOwners().contains(RobotListModel.getInstance().getRobotModelByName(name))){
					possibilities.remove(name);
					System.out.println("Remove!");
				}
			}
			
			if (possibilities.size() == 0) return;
			
			String s = (String) JOptionPane.showInputDialog(panel,
					"Select a new owner for task: " + task.getName(),
					"Add owner", JOptionPane.PLAIN_MESSAGE, null,
					possibilities.toArray(), possibilities.get(0));

			// If a string was returned, say so.
			if ((s != null) && (s.length() > 0)) {
				task.addOwner(RobotListModel.getInstance().getRobotModelByName(
						s));
				if (lblOwners.getText().equals("Owners: ")) {
					lblOwners.setText("Owners: " + s);
				} else {
					lblOwners.setText(lblOwners.getText() + ", " + s);
				}
				panel.repaint();
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
								panel,
								"Select an owner to remove for task: "
										+ task.getName(), "Remove owner",
								JOptionPane.PLAIN_MESSAGE, null, possibilities,
								possibilities[0]);

				// If a string was returned, say so.
				if ((s != null) && (s.length() > 0)) {
					task.removeOwner(RobotListModel.getInstance()
							.getRobotModelByName(s));
					String stringOfOwners = "Owners: ";
					List<RobotModel> owners = task.getOwners();
					for (int i = 0; i < owners.size(); i++) {
						stringOfOwners += owners.get(i).getName()
								+ ((i == (owners.size() - 1)) ? ("") : (", "));
					}
					lblOwners.setText(stringOfOwners);
					panel.repaint();
					return;
				}
			}
		} else if (item.getText().equals("Remove")) {
			TaskListModel.getInstance().removeTask(task);
			return;
		}
	}

	class PopupListener extends MouseAdapter {
		JPopupMenu popup;

		PopupListener(JPopupMenu popupMenu) {
			popup = popupMenu;
		}

		public void mousePressed(MouseEvent e) {
			maybeShowPopup(e);
		}

		public void mouseReleased(MouseEvent e) {
			maybeShowPopup(e);
		}

		private void maybeShowPopup(MouseEvent e) {
			if (e.isPopupTrigger()) {
				popup.show(e.getComponent(), e.getX(), e.getY());
			}
		}
	}
}
