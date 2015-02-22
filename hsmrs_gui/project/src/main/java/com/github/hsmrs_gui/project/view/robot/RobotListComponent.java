package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.Color;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.MatteBorder;

import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.RenderableComponent;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.view.buttons.TransparentButton;
import src.main.java.com.github.hsmrs_gui.project.view.list.*;
import net.miginfocom.swing.MigLayout;

public class RobotListComponent extends RenderableComponent<RobotModel> {

	private JPanel panel;
	private JLabel lblColor;
	private JLabel lblName;
	private JLabel lblStatus;
	private JLabel lblTask;
	private RobotModel robot;

	public RobotListComponent() {
	}

	@Override
	public JComponent create(ListItem<RobotModel> listItem) {
		robot = listItem.getListObject();

		panel = new JPanel(new MigLayout("fill", "[]push[]"));
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor
				: (robot.getNeedsHelp() ? Color.orange : Color.WHITE));
		panel.setBorder(new MatteBorder(0, 0, 1, 0, Color.GRAY));

		lblColor = new JLabel();
		lblColor.setBackground(robot.getColor());
		lblColor.setOpaque(true);

		String nameString = robot.getName();
		if (robot.getRole() != null){
			nameString += " - " + robot.getRole().getName();
		}
		lblName = new JLabel(nameString, JLabel.CENTER);
		lblName.setOpaque(false);
		
		lblStatus = new JLabel("Status: " + robot.getStatus());
		lblStatus.setOpaque(false);

		lblTask = new JLabel("Task: " + robot.getAssignedTask().getType(), JLabel.CENTER);
		lblTask.setOpaque(false);

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

		panel.add(lblColor, "split, width 10:10:10, height 10:10:10");
		panel.add(lblName, "growx, span 2, wrap");
		panel.add(lblStatus, "growx, span 2, wrap");
		panel.add(lblTask);
		// panel.add(expandButton, "alignx right");

		return panel;
	}

	@Override
	public void update(ListItem<RobotModel> listItem) {
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor
				: (robot.getNeedsHelp() ? Color.orange : Color.WHITE));

		String nameString = robot.getName();
		if (robot.getRole() != null){
			nameString += " - " + robot.getRole().getName();
		}
		lblName = new JLabel(nameString, JLabel.CENTER);
		lblName.setOpaque(false);
		
		lblStatus = new JLabel("Status: " + robot.getStatus());
		lblStatus.setOpaque(false);

		lblTask = new JLabel("Task: " + robot.getAssignedTask().getType(), JLabel.CENTER);
		lblTask.setOpaque(false);
		
		panel.removeAll();

		panel.add(lblColor, "split, width 10:10:10, height 10:10:10");
		panel.add(lblName, "growx, span 2, wrap");
		panel.add(lblStatus, "growx, span 2, wrap");
		panel.add(lblTask);

		// if(!listItem.isExpanded()) {
		// panel.add(endDateLabel);
		// expandButton.reset();
		// panel.add(expandButton, "alignx right");
		// } else {
		// panel.add(endDateLabel, "span 2, wrap");
		// panel.add(isTeamLabel, "span 2, wrap");
		// if(hasDescription) panel.add(descriptionText, "growx, span 2, wrap");
		// panel.add(createdByLabel);
		// compressButton.reset();
		// panel.add(compressButton, "alignx right");
		// }

		panel.repaint();

	}
}
