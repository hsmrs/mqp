package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.Color;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.MatteBorder;

import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.RenderableComponent;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.view.buttons.TransparentButton;
import src.main.java.com.github.hsmrs_gui.project.view.list.*;
import net.miginfocom.swing.MigLayout;

public class RobotListComponent extends RenderableComponent<RobotModel>{

	JPanel panel;
	JLabel lblName;
	JLabel lblTask;
	
	public RobotListComponent() {
	}

	@Override
	public JComponent create(ListItem<RobotModel> listItem) {
		RobotModel robot = listItem.getListObject();

		panel = new JPanel(new MigLayout("fill", "[]push[]"));
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor : Color.WHITE);
		panel.setBorder(new MatteBorder(0, 0, 1, 0, Color.GRAY));

		lblName = new JLabel(robot.getName(), JLabel.CENTER);
		lblName.setOpaque(false);
		
		lblTask = new JLabel(robot.getAssignedTask().getName(), JLabel.CENTER);
		lblTask.setOpaque(false);


		//compressButton = new TransparentButton(compress);
		//compressButton.setMargin(new Insets(0, 0, 0 ,0));
		//compressButton.setActionCommand("compress");
		//compressButton.addActionListener(new ListItemExpandListener<Event>(listItem));

		//expandButton = new TransparentButton(expand);
		//expandButton.setMargin(new Insets(0, 0, 0 ,0));
		//expandButton.setActionCommand("expand");
		//expandButton.addActionListener(new ListItemExpandListener<Event>(listItem));
		
		panel.add(lblName, "growx, span 2, wrap");
		panel.add(lblTask);
		//panel.add(expandButton, "alignx right");

		return panel;
	}

	@Override
	public void update(ListItem<RobotModel> listItem) {
		panel.setBackground(listItem.isSelected() ? Colors.selectionColor : Color.WHITE);

		if(listItem.isDirty()) {
			panel.removeAll();

			panel.add(lblName, "growx, span 2, wrap");
			panel.add(lblTask);

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
