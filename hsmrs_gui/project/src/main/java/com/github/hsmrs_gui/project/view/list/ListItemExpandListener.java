package src.main.java.com.github.hsmrs_gui.project.view.list;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class ListItemExpandListener<T> implements ActionListener {

	private ListItem<T> listItem;
	
	/**
	 * This is the constructor for the ListItemExpandListener.
	 * @param listItem The ListItem that will be expanded.
	 */
	public ListItemExpandListener(ListItem<T> listItem) {
		this.listItem = listItem;
	}
	
	/**
	 * This method is called whenever an expand action is executed on the given ListItem.
	 */
	@Override
	public void actionPerformed(ActionEvent e) {
		if(e.getActionCommand().equals("expand")) listItem.setExpand(true);
		else if(e.getActionCommand().equals("compress")) listItem.setExpand(false);
		listItem.requestUpdate();
	}
}
