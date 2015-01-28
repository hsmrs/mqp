package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.Color;
import java.awt.Font;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;

import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RoleModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecificationListModel;
import net.miginfocom.swing.MigLayout;


public class NewRolePanel extends JPanel implements ListSelectionListener{

	private JLabel lblTitle;
	private JLabel lblName;
	private JTextField txtName;
	private JScrollPane typeScrollView;
	private JList typeListView;
	private JButton btnNewType;
	private JLabel lblOwner;
	private JButton btnCreate;
	private JButton btnCancel;

	public NewRolePanel(){
		lblTitle = new JLabel("New Role", JLabel.CENTER);
		lblTitle.setBackground(Color.decode("0XC0BCB6"));
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));

		lblName = new JLabel("Name");
		
		txtName = new JTextField();

		TaskSpecificationListModel tslm = TaskSpecificationListModel
				.getInstance();
		List<String> taskNames = tslm.getSpecNames();
		String[] taskNameArray = new String[taskNames.size()];
		taskNames.toArray(taskNameArray);
		typeListView = new JList(taskNameArray);
		typeListView.setSelectedIndex(0);
		ListSelectionModel listSelectionModel = typeListView
				.getSelectionModel();
		listSelectionModel.addListSelectionListener(this);

		typeScrollView = new JScrollPane(typeListView);
		
		btnCreate = new JButton("Create");
		btnCreate.addActionListener(RobotController.getInstance());
		btnCreate.setActionCommand("Create New Role");

		btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(RobotController.getInstance());
		btnCancel.setActionCommand("Cancel New Role");

		// this.setBackground(Color.white);
		this.setLayout(new MigLayout("insets 0, debug", "[left, 45%]10%[right, 45%]",
				"[]20[][]20[]"));
		// this.setLayout(new MigLayout("fill, debug", "[][]", "[][]"));

		this.add(lblTitle, "center, span, growx, top, wrap");
		this.add(lblName, "gapleft 10, split");
		this.add(txtName, "span, width 120:120:120, wrap");
		this.add(typeScrollView, "gapleft 10, gapright 10, grow, span, wrap");
		this.add(btnCreate, "id assign, center, split");
		this.add(btnCancel, "center, skip");
	}

	@Override
	public void valueChanged(ListSelectionEvent e) {
		// TODO Auto-generated method stub
		
	}

	public RoleModel getNewRole() {
		String name = txtName.getText();
		Object[] taskTypesObjs = typeListView.getSelectedValues();
		List<String> taskTypes = new ArrayList<String>();
		for (Object obj : taskTypesObjs){
			taskTypes.add(String.valueOf(obj));
		}
		List<TaskSpecification> tasks = new ArrayList<TaskSpecification>();
		for (String taskType : taskTypes){
			TaskSpecification taskSpec = TaskSpecificationListModel.getInstance().getSpecByName(taskType);
			tasks.add(taskSpec);
		}
		RoleModel newRole = new RoleModel(name, tasks);
		return newRole;
	}
}
