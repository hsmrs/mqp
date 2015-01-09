package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.task.Task;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecificationListModel;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import net.miginfocom.swing.MigLayout;

public class NewTaskPanel extends JPanel{

	private JLabel lblTitle;
	private JLabel lblType;
	private JScrollPane typeScrollView;
	private JList typeListView;
	private JButton btnNewType;
	private JLabel lblParam;
	private JScrollPane paramScrollView;
	private JPanel paramPane;
	private JButton btnCreate;
	private JButton btnCancel;
	
	public NewTaskPanel(){
		System.out.println("New Task Panel");
		
		lblTitle = new JLabel("New Task", JLabel.CENTER);
		lblTitle.setBackground(Color.decode("0XC0BCB6"));
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));
		
		lblType = new JLabel("Type");
		
		TaskSpecificationListModel tslm = TaskSpecificationListModel.getInstance();
		List<String> specNames = tslm.getSpecNames();
		specNames.add(tslm.getElementAt(0).getName());
		String[] specNameArray = new String[specNames.size()];
		specNames.toArray(specNameArray);
		typeListView = new JList(specNameArray);
		typeScrollView = new JScrollPane(typeListView);
		
		btnNewType = new JButton("New type");
		btnNewType.addActionListener(TaskController.getInstance());
		btnNewType.setActionCommand("New task type");
		
		lblParam = new JLabel("Parameters");
		
		TaskSpecification defaultSpec = tslm.getElementAt(0);
		paramPane = generateParamPanel(defaultSpec);
		paramScrollView = new JScrollPane(paramPane);
		
		btnCreate = new JButton("Create");
		btnCreate.addActionListener(TaskController.getInstance());
		btnCreate.setActionCommand("Create task");
		
		btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(TaskController.getInstance());
		btnCancel.setActionCommand("Cancel new task");
		
		//this.setBackground(Color.white);
		this.setLayout(new MigLayout("insets 0, fill, debug", "[left, fill, 45%]10%[right, fill, 45%]", "[]20[][][]20[][]20[]"));
		
		this.add(lblTitle, "center, span, growx, wrap");
		this.add(lblType, "gapleft 10, span, wrap");
		this.add(typeScrollView, "gapleft 10, gapright 10, grow, span, wrap");
		this.add(btnNewType, "gapleft 10, wrap");
		this.add(lblParam, "gapleft 10, span, wrap");
		this.add(paramScrollView, "gapleft 10, gapright 10, grow, span, wrap");
		this.add(btnCreate, "gapleft 10, center");
		this.add(btnCancel, "gapright 10, center");
	}
	
	private JPanel generateParamPanel(TaskSpecification spec){
		
		return new JPanel();
	}
}
