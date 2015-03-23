package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.Color;
import java.awt.Font;
import java.util.List;

import javax.management.relation.RoleList;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.ScrollPaneConstants;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;

import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RoleListModel;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RoleModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecificationListModel;
import net.miginfocom.swing.MigLayout;

public class RoleListPanel extends JPanel implements ListSelectionListener{
	
	private JLabel lblTitle;
	private JLabel lblType;
	private JScrollPane typeScrollView;
	private JList typeListView;
	private JButton btnNewType;
	private JLabel lblOwner;
	private JButton btnAssign;
	private JButton btnCancel;

	/**
	 * The constructor for the RoleListPanel class.
	 */
	public RoleListPanel(){
		lblTitle = new JLabel("Role List", JLabel.CENTER);
		lblTitle.setBackground(Color.decode("0XC0BCB6"));
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));

		lblType = new JLabel("Role");

		RoleListModel tslm = RoleListModel
				.getInstance();
		List<String> roleNames = tslm.getRoleNames();
		String[] roleNameArray = new String[roleNames.size()];
		roleNames.toArray(roleNameArray);
		typeListView = new JList(roleNameArray);
		typeListView.setSelectedIndex(0);
		ListSelectionModel listSelectionModel = typeListView
				.getSelectionModel();
		listSelectionModel.addListSelectionListener(this);

		typeScrollView = new JScrollPane(typeListView);

		btnNewType = new JButton("New Role");
		btnNewType.addActionListener(RobotController.getInstance());
		btnNewType.setActionCommand("New Role");
		
		btnAssign = new JButton("Assign");
		btnAssign.addActionListener(RobotController.getInstance());
		btnAssign.setActionCommand("Assign Role");

		btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(RobotController.getInstance());
		btnCancel.setActionCommand("Cancel Role");

		// this.setBackground(Color.white);
		this.setLayout(new MigLayout("insets 0", "[left, 45%]10%[right, 45%]",
				"[]20[][][]20[]"));
		// this.setLayout(new MigLayout("fill, debug", "[][]", "[][]"));

		this.add(lblTitle, "center, span, growx, top, wrap");
		this.add(lblType, "gapleft 10, span, wrap");
		this.add(typeScrollView, "gapleft 10, gapright 10, grow, span, wrap");
		this.add(btnNewType, "x assign.x, wrap");
		this.add(btnAssign, "id assign, gapleft 10, center");
		this.add(btnCancel, "gapright 10, center");
	}

	@Override
	public void valueChanged(ListSelectionEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Gets the Role which is currently selected.
	 * @return The Role which is currently selected.
	 */
	public RoleModel getSelectedRole() {
		return RoleListModel.getInstance().
				getRole(typeListView.getSelectedIndex());
	}
}
