package src.main.java.com.github.hsmrs_gui.project.view.robot;

import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItemRenderer;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;

public class RobotListItemRenderer implements ListItemRenderer<RobotModel> {
	
	public RobotListItemRenderer() {
	}

	@Override
	public void createRenderedListComponents(SRList<RobotModel> listPanel,
			List<ListItem<RobotModel>> listItems) {
		for (ListItem<RobotModel> listItem : listItems) {
			RobotListComponent renderableComponent = new RobotListComponent();
			listItem.setRenderableComponent(renderableComponent);
			listItem.setComponent(renderableComponent.create(listItem));
		}
		
	}

	@Override
	public void updateRenderedListComponents(SRList<RobotModel> listPanel,
			List<ListItem<RobotModel>> listItems) {
		for (ListItem<RobotModel> listItem : listItems) {
			listItem.update();
		}
		
	}
}
