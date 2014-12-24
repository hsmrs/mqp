package src.main.java.com.github.hsmrs_gui.project.view.list;

import java.awt.Color;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class DefaultListItemRenderer<T> implements ListItemRenderer<T> {
	@Override
	public void createRenderedListComponents(SRList<T> listPanel, List<ListItem<T>> listItems) {
		for(ListItem<T> listItem: listItems) {
			DefaultListItem defaultListItem = new DefaultListItem();
			listItem.setRenderableComponent(defaultListItem);
			listItem.setComponent(defaultListItem.create(listItem));
		}
	}

	@Override
	public void updateRenderedListComponents(SRList<T> listPanel, List<ListItem<T>> listItems) {
		for(ListItem<T> listItem: listItems)
			listItem.getRenderableComponent().update(listItem);
	}

	class DefaultListItem extends RenderableComponent<T> {

		private JPanel p;

		@Override
		public JComponent create(ListItem<T> listItem) {
			p = new JPanel();
			JLabel label = new JLabel("Default List Item");
			p.add(label);
			return p;
		}

		@Override
		public void update(ListItem<T> listItem) {
		}
	}
}
