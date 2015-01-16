package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.awt.Color;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.ScrollPaneConstants;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;

import src.main.java.com.github.hsmrs_gui.project.controller.TaskController;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecification;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskSpecificationListModel;
import src.main.java.com.github.hsmrs_gui.project.util.VerticalLabelUI;
import net.miginfocom.swing.MigLayout;

public class NewTaskTypePanel extends JPanel {
	private JLabel lblTitle;
	private JLabel lblName;
	private JTextField txtName;
	private JTabbedPane subtaskTabPane;
	private JButton btnNewSubtask;
	private JButton btnCreate;
	private JButton btnCancel;
	private ArrayList<JLabel[]> tabLblArrays;
	private ArrayList<JTextField[]> tabTxtArrays;
	
	private int numSubTasks = 1;
	private int maxTabIndex = 0;

	public NewTaskTypePanel() {
		tabLblArrays = new ArrayList<JLabel[]>();
		tabTxtArrays = new ArrayList<JTextField[]>();
		
		lblTitle = new JLabel("New Task Type", JLabel.CENTER);
		lblTitle.setBackground(Color.decode("0XC0BCB6"));
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));

		lblName = new JLabel("Name");

		txtName = new JTextField();

		subtaskTabPane = new JTabbedPane(JTabbedPane.LEFT);
		subtaskTabPane.add("a", generateSubtaskPane(numSubTasks));
		//JLabel lblVert = new JLabel("Subtask " + numSubTasks);
		//lblVert.setUI(new VerticalLabelUI(false)); // true/false to make it upwards/downwards
		//subtaskTabPane.setTabComponentAt(maxTabIndex, lblVert); // For component1
		
		btnNewSubtask = new JButton("New Subtask");
		btnNewSubtask.addActionListener(TaskController.getInstance());
		btnNewSubtask.setActionCommand("New subtask");

		btnCreate = new JButton("Create");
		btnCreate.addActionListener(TaskController.getInstance());
		btnCreate.setActionCommand("Create task");

		btnCancel = new JButton("Cancel");
		btnCancel.addActionListener(TaskController.getInstance());
		btnCancel.setActionCommand("Cancel new task");

//		this.setLayout(new MigLayout("insets 0", "[left, 45%]10%[right, 45%]",
//				"[]20[]20[]push[]20[]push"));
		this.setLayout(new MigLayout("insets 0, fill", "[][]",
				"[]20[]20[push]push[]20[]push"));
		// this.setLayout(new MigLayout("fill, debug", "[][]", "[][]"));

		this.add(lblTitle, "center, pushx, growx, span, top, wrap");
		this.add(lblName, "gapleft 10");
		this.add(txtName, "width 120:120:120, gapright 10, align left, wrap");
		this.add(subtaskTabPane, "push, grow, span, wrap");
		this.add(btnNewSubtask, "x btn.x, left, span, wrap");
		this.add(btnCreate, "id btn, gapleft 10, center");
		this.add(btnCancel, "gapright 10, center");
	}
	
	private JPanel generateSubtaskPane(final int index){
		JLabel lblType;
		JScrollPane typeScrollView;
		JList typeListView;
		JLabel lblParam;
		JScrollPane paramScrollView;
		JPanel paramPane;
		JButton btnDelete;
		JPanel returnPanel;
		
		lblType = new JLabel("Type");

		TaskSpecificationListModel tslm = TaskSpecificationListModel
				.getInstance();
		List<String> specNames = tslm.getSpecNames();
		String[] specNameArray = new String[specNames.size()];
		specNames.toArray(specNameArray);
		typeListView = new JList(specNameArray);
		typeListView.setSelectedIndex(0);
		
		typeScrollView = new JScrollPane(typeListView);

		lblParam = new JLabel("Parameters");

		TaskSpecification defaultSpec = tslm.getElementAt(0);
		paramPane = generateParamPanel(defaultSpec);
		//paramPane = new JPanel();
		paramScrollView = new JScrollPane(paramPane);
		paramScrollView
				.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		
		ListSelectionModel listSelectionModel = typeListView.getSelectionModel();
		listSelectionModel.addListSelectionListener(new TaskTypeListListener(paramScrollView));
		
		btnDelete = new JButton("Delete");
		btnDelete.addActionListener(new ActionListener() {
			
			public void actionPerformed(ActionEvent arg0) {
				removeSubtask(index);
			}
		});
		
		returnPanel = new JPanel();
		returnPanel.setLayout(new MigLayout("insets 10", "[left, 45%]10%[right, 45%]",
				"[][]20[][]20[]"));
//		
		returnPanel.add(lblType, "span, wrap");
		returnPanel.add(typeScrollView, "grow, span, wrap");
		returnPanel.add(lblParam, "span, wrap");
//		returnPanel.add(paramScrollView, "grow, span, wrap");
		returnPanel.add(paramPane, "grow, span, wrap");
		returnPanel.add(btnDelete, "center, span");

		return returnPanel;
		
	}
	
	private JPanel generateParamPanel(TaskSpecification spec){
		JPanel returnPanel = new JPanel();
		JLabel[] paramlbls;
		JTextField[] paramtxts;
		
		returnPanel.setLayout(new MigLayout());
		
		List<String> paramTypePairs = spec.getParameterTypePairs();
		paramlbls = new JLabel[paramTypePairs.size()];
		paramtxts = new JTextField[paramTypePairs.size()];
		
		System.out.println(tabLblArrays);
		for (int i = 0; i < paramTypePairs.size(); i++){
			String param = paramTypePairs.get(i).split(":")[0];
			paramlbls[i] = new JLabel(param);
			paramtxts[i] = new JTextField();
			
			returnPanel.add(paramlbls[i], "left");
			returnPanel.add(paramtxts[i], "gapleft 20, width 80:80:80, wrap");
			//returnPanel.validate();
		}
		
		tabLblArrays.add(paramlbls);
		tabTxtArrays.add(paramtxts);
		//returnPanel.validate();
		
		return returnPanel;
	}
	
	public void addNewSubtask(){
		numSubTasks++;
		maxTabIndex++;
		subtaskTabPane.add("a", generateSubtaskPane(numSubTasks));
		//JLabel lblVert = new JLabel("Subtask " + numSubTasks);
		//lblVert.setUI(new VerticalLabelUI(false)); // true/false to make it upwards/downwards
		//subtaskTabPane.setTabComponentAt(maxTabIndex, lblVert);
		//revalidate();
		//repaint();
	}
	
	public void removeSubtask(int index){
		subtaskTabPane.remove(index);
	}
	
	public class TaskTypeListListener implements ListSelectionListener {
		
		JScrollPane paramScrollPane;
		
		public TaskTypeListListener(JScrollPane paramScrollPane){
			this.paramScrollPane = paramScrollPane;
		}

		public void valueChanged(ListSelectionEvent e) {
			int index = e.getFirstIndex();
			
			TaskSpecification ts = TaskSpecificationListModel.getInstance().getElementAt(index);
	        JPanel p = generateParamPanel(ts);
	        paramScrollPane = new JScrollPane(p);
			paramScrollPane.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
			//validate();
		}
		
	}
}
