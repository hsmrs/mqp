package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.MatteBorder;

import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.RenderableComponent;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.view.buttons.TransparentButton;
import src.main.java.com.github.hsmrs_gui.project.view.list.*;
import net.miginfocom.swing.MigLayout;

public class TaskListComponent extends RenderableComponent<TaskModel>{

	JPanel panel;
	JLabel lblName;
	JLabel lblOwners;
	JLabel lblStatus;
	
	public TaskListComponent() {
	}

	@Override
	public JComponent create(ListItem<TaskModel> listItem) {
		TaskModel task = listItem.getListObject();

		panel = new JPanel(new MigLayout("fill", "[]", "[]10[]10[]"));
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor : Color.WHITE);
		panel.setBorder(new MatteBorder(0, 0, 1, 0, Color.GRAY));

		lblName = new JLabel(task.getName());
		lblName.setOpaque(false);
		
		lblStatus = new JLabel("Status: " + task.getStatus());
		lblName.setOpaque(false);
		
		String stringOfOwners = "Owners: ";
		List<RobotModel> owners = task.getOwners();
		for (int i = 0; i < owners.size(); i++){
			stringOfOwners += owners.get(i).getName() + ((i == (owners.size() - 1)) ? ("") : (", "));
		}
		lblOwners = new JLabel(stringOfOwners);
		lblOwners.setOpaque(false);


		//compressButton = new TransparentButton(compress);
		//compressButton.setMargin(new Insets(0, 0, 0 ,0));
		//compressButton.setActionCommand("compress");
		//compressButton.addActionListener(new ListItemExpandListener<Event>(listItem));

		//expandButton = new TransparentButton(expand);
		//expandButton.setMargin(new Insets(0, 0, 0 ,0));
		//expandButton.setActionCommand("expand");
		//expandButton.addActionListener(new ListItemExpandListener<Event>(listItem));
		
		panel.add(lblName, "pushx, growx, wrap");
		panel.add(lblStatus, "pushx, growx, wrap");
		panel.add(lblOwners);
		//panel.add(expandButton, "alignx right");

		return panel;
	}

	@Override
	public void update(ListItem<TaskModel> listItem) {
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor : Color.WHITE);

		if(listItem.isDirty()) {
			panel.removeAll();

			panel.add(lblName, "pushx, growx, wrap");
			panel.add(lblStatus, "pushx, growx, wrap");
			panel.add(lblOwners);

//			if(!listItem.isExpanded()) {
//				panel.add(endDateLabel);
//				expandButton.reset();
//				panel.add(expandButton, "alignx right");
//			} else {
//				panel.add(endDateLabel, "span 2, wrap");
//				panel.add(isTeamLabel, "span 2, wrap");
//				if(hasDescription) panel.add(descriptionText, "growx, span 2, wrap");
//				panel.add(createdByLabel);
//				compressButton.reset();
//				panel.add(compressButton, "alignx right");
//			}
			
			panel.repaint();
		}
	}
}
