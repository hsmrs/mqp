package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BorderFactory;
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

import src.main.java.com.github.hsmrs_gui.project.Globals;
import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotListModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecificationListModel;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import net.miginfocom.swing.MigLayout;

public class NewTaskPanel extends JPanel implements ListSelectionListener {

	private JLabel lblTitle;
	private JLabel lblType;
	private JScrollPane typeScrollView;
	private JList typeListView;
	private JButton btnNewType;
	private JLabel lblParam;
	private JScrollPane paramScrollView;
	private JPanel paramPane;
	private JLabel lblOwner;
	private JComboBox ownerSelBox;
	private JButton btnCreate;
	private JButton btnCancel;
	private JLabel[] paramlbls;
	private JTextField[] paramtxts;

	public NewTaskPanel() {

		lblTitle = new JLabel("New Task", JLabel.CENTER);
		lblTitle.setBackground(Color.decode("0XC0BCB6"));
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));

		lblType = new JLabel("Type");

		TaskSpecificationListModel tslm = TaskSpecificationListModel
				.getInstance();
		List<String> specNames = tslm.getSpecNames();
		// specNames.add(tslm.getElementAt(0).getName());
		String[] specNameArray = new String[specNames.size()];
		specNames.toArray(specNameArray);
		typeListView = new JList(specNameArray);
		typeListView.setSelectedIndex(0);
		ListSelectionModel listSelectionModel = typeListView
				.getSelectionModel();
		listSelectionModel.addListSelectionListener(this);

		typeScrollView = new JScrollPane(typeListView);

		btnNewType = new JButton("New type");
		btnNewType.addActionListener(TaskController.getInstance());
		btnNewType.setActionCommand("New task type");

		lblParam = new JLabel("Parameters");

		TaskSpecification defaultSpec = tslm.getElementAt(0);
		paramPane = generateParamPanel(defaultSpec);
		paramPane.validate();
		paramScrollView = new JScrollPane(paramPane);
		paramScrollView
				.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);

		lblOwner = new JLabel("Owner:");
		
		List<String> robotList = RobotListModel.getInstance().getRobotNames();
		robotList.add(0, "None");
		String[] robotArray = robotList.toArray(new String[0]);
		ownerSelBox = new JComboBox(robotArray);
		
		btnCreate = new JButton("Create");
		btnCreate.addActionListener(TaskController.getInstance());
		btnCreate.setActionCommand("Create task");

		btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(TaskController.getInstance());
		btnCancel.setActionCommand("Cancel new task");

		// this.setBackground(Color.white);
		this.setLayout(new MigLayout("insets 0", "[left, 45%]10%[right, 45%]",
				"[]20[][][]20[][]20[]20[]"));
		// this.setLayout(new MigLayout("fill, debug", "[][]", "[][]"));

		this.add(lblTitle, "center, span, growx, top, wrap");
		this.add(lblType, "gapleft 10, span, wrap");
		this.add(typeScrollView, "gapleft 10, gapright 10, grow, span, wrap");
		this.add(btnNewType, "gapleft 10, wrap");
		this.add(lblParam, "gapleft 10, span, wrap");
		this.add(paramScrollView, "gapleft 10, gapright 10, grow, span, wrap");
		this.add(lblOwner, "gapleft 10");
		this.add(ownerSelBox, "gapright 10, growx, wrap");
		this.add(btnCreate, "gapleft 10, center");
		this.add(btnCancel, "gapright 10, center");
	}

	private JPanel generateParamPanel(TaskSpecification spec) {
		System.out.println("Generating parameter panel for " + spec.getName());
		JPanel returnPanel = new JPanel();
		returnPanel.setLayout(new MigLayout("", "[]30[]", "[]"));

		List<String> paramTypePairs = spec.getParameterTypePairs();
		paramlbls = new JLabel[paramTypePairs.size()];
		paramtxts = new JTextField[paramTypePairs.size()];
		for (int i = 0; i < paramTypePairs.size(); i++) {
			String param = paramTypePairs.get(i).split(":")[0];
			paramlbls[i] = new JLabel(param);
			paramtxts[i] = new JTextField();

			returnPanel.add(paramlbls[i], "left");
			returnPanel.add(paramtxts[i], "width 120:120:120, wrap");
		}
		
		returnPanel.validate();
		
		return returnPanel;
	}

	public TaskSpecification getNewTaskSpec() {
		int selIndex = typeListView.getSelectedIndex();
		System.out.println("Index = " + selIndex);
		return TaskSpecificationListModel.getInstance().getElementAt(selIndex);
	}

	public List<String> getNewTaskParamValues() {
		ArrayList<String> paramValues = new ArrayList<String>();
		for (JTextField txt : paramtxts) {
			String text = txt.getText();
			if (text.contains("$")){
				String splitText = text.split("$")[0].substring(1);
				String varName = splitText.split(" ")[0];
				
				if (!Globals.gui_vars.containsKey(varName)){
					System.out.println("ERROR! Variable: " + varName + " not found!");
					continue;
				}
				try{
				paramValues.add(Globals.gui_vars.get(varName).toString());
				}
				catch(NullPointerException e)
				{
					System.out.println("ERROR: Variable: " + varName + " has no value!");
				}
			}else{
				paramValues.add(txt.getText());
			}
		}
		return paramValues;
	}
	
	public String getNewTaskOwner(){
		return (String)ownerSelBox.getSelectedItem();
	}

	public void valueChanged(ListSelectionEvent e) {
		if (e.getValueIsAdjusting()) {
			ListSelectionModel lsm = (ListSelectionModel) e.getSource();

			System.out.println("Value Changed");
			//int index = e.getFirstIndex();
			int index = typeListView.getSelectedIndex();

			TaskSpecification ts = TaskSpecificationListModel.getInstance()
					.getElementAt(index);
			JPanel p = generateParamPanel(ts);
			paramScrollView.setViewportView(p);
			//paramScrollView = new JScrollPane(p);
			paramScrollView
					.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
			paramScrollView.revalidate();
			paramScrollView.repaint();
			revalidate();
			repaint();
		}
	}
}
