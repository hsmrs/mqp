package src.main.java.com.github.hsmrs_gui.project.view.task;

import java.util.List;

import src.main.java.com.github.hsmrs_gui.project.view.list.ListItem;
import src.main.java.com.github.hsmrs_gui.project.view.list.ListItemRenderer;
import src.main.java.com.github.hsmrs_gui.project.view.list.SRList;
import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;
import src.main.java.com.github.hsmrs_gui.project.model.Task;

public class TaskListItemRenderer implements ListItemRenderer<Task> {
	
	public TaskListItemRenderer() {
	}

	@Override
	public void createRenderedListComponents(SRList<Task> listPanel,
			List<ListItem<Task>> listItems) {
		for (ListItem<Task> listItem : listItems) {
			TaskListComponent renderableComponent = new TaskListComponent();
			listItem.setRenderableComponent(renderableComponent);
			listItem.setComponent(renderableComponent.create(listItem));
		}
		
	}

	@Override
	public void updateRenderedListComponents(SRList<Task> listPanel,
			List<ListItem<Task>> listItems) {
		for (ListItem<Task> listItem : listItems) {
			listItem.update();
		}
		
	}
}
