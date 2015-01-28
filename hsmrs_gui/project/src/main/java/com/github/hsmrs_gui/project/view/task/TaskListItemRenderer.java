package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItemRenderer;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.task.TaskModel;

public class TaskListItemRenderer implements ListItemRenderer<TaskModel> {
	
	public TaskListItemRenderer() {
	}

	@Override
	public void createRenderedListComponents(SRList<TaskModel> listPanel,
			List<ListItem<TaskModel>> listItems) {
		for (ListItem<TaskModel> listItem : listItems) {
			TaskListComponent renderableComponent = new TaskListComponent();
			listItem.setRenderableComponent(renderableComponent);
			listItem.setComponent(renderableComponent.create(listItem));
		}
		
	}

	@Override
	public void updateRenderedListComponents(SRList<TaskModel> listPanel,
			List<ListItem<TaskModel>> listItems) {
		for (ListItem<TaskModel> listItem : listItems) {
			listItem.update();
		}
		
	}
}
