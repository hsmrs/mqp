package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.awt.Color;
import java.awt.Font;
import java.awt.Point;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.ListModel;

import src.main.java.com.github.hsmrs_gui.project.controller.RobotController;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.util.Colors;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItemListener;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import src.main.java.com.github.hsmrs_gui.project.view.task.TaskListComponentPopup;
import net.miginfocom.swing.MigLayout;

public class RobotListView extends JPanel implements ListItemListener<RobotModel>{
	
	private static RobotListView instance;
	private JLabel lblTitle;
	private SRList<RobotModel> robotListView;
	
	private RobotListView (){
	}
	
	/**
	 * Returns the only existing instance of the RobotListView.
	 * @return The only existing instance of the RobotListView.
	 */
	public static RobotListView getInstance(){
		if (instance == null) {
			instance = new RobotListView();
		}
		return instance;
	}
	
	/**
	 * Sets the underlying ListModel for this view.
	 * @param listModel The ListModel to be associated with this view.
	 */
	public void setListModel(ListModel listModel) {
		lblTitle = new JLabel("Robot List", JLabel.CENTER);
		lblTitle.setBackground(Colors.bannerColor);
		lblTitle.setOpaque(true);
		lblTitle.setFont(new Font(lblTitle.getFont().getName(), Font.PLAIN, 24));
		
		robotListView = new SRList<RobotModel>(listModel, new RobotListItemRenderer());
		robotListView.addListItemListener(this);
		this.setBackground(Color.white);
		this.setBorder(BorderFactory.createMatteBorder(1, 3, 3, 1, Color.black));
		this.setLayout(new MigLayout("insets 0", "[left, fill]", "[]0[fill]"));
		this.add(lblTitle, "wrap");
		this.add(robotListView, "push, growy, wrap");
	}
	
	/**
	 * Gets the first selected robot in the view. This is considered the robot in focus.
	 * @return The robot in focus.
	 */
	public RobotModel getRobotInFocus(){
		if (robotListView != null && robotListView.getSelectedItems().size() > 0){
			List<RobotModel> selectedRobots = robotListView.getSelectedItems();
			return selectedRobots.get(0);
		}else {
			return null;
		}
	}
	
	/**
	 * Updates the component views within this list.
	 */
	public void update(){
		robotListView.updateComponents();
	}

	@Override
	public void itemsSelected(List<RobotModel> listObjects) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void itemDoubleClicked(RobotModel listObject) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void itemRightClicked(RobotModel listObject, Point p) {
		RobotListComponentPopup popup = new RobotListComponentPopup(listObject, this);
		popup.show(this, p.x, p.y);
	}

	@Override
	public void itemFocused(RobotModel listObject) {
		// TODO Auto-generated method stub
		
	}

}
