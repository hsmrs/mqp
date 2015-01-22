package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.Color;
import java.awt.Font;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.ListModel;

import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import net.miginfocom.swing.MigLayout;

public class RobotListView extends JPanel{
	
	private static RobotListView instance;
	private JLabel lblTitle;
	private SRList<RobotModel> robotListView;
	
	private RobotListView (){
		RobotController.getInstance().setRobotListView(this);
	}
	
	public static RobotListView getInstance(){
		if (instance == null) {
			instance = new RobotListView();
		}
		return instance;
	}
	
	public void setListModel(ListModel listModel) {
		lblTitle = new JLabel("Robot List", JLabel.CENTER);
		lblTitle.setBackground(Colors.bannerColor);
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));
		
		robotListView = new SRList<RobotModel>(listModel, new RobotListItemRenderer());
		
		this.setBackground(Color.white);
		this.setBorder(BorderFactory.createMatteBorder(1, 3, 3, 1, Color.black));
		this.setLayout(new MigLayout("insets 0", "[left, fill]", "[]0[fill]"));
		this.add(lblTitle, "wrap");
		this.add(robotListView, "push, growy, wrap");
	}
	
	public RobotModel getRobotInFocus(){
		if (robotListView != null && robotListView.getSelectedItems().size() > 0){
			List<RobotModel> selectedRobots = robotListView.getSelectedItems();
			return selectedRobots.get(0);
		}else {
			return null;
		}
	}
	
	public void update(){
		robotListView.updateComponents();
	}

}
