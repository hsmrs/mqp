package src.main.java.com.github.hsmrs_gui.project.view.list;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class ListItemExpandListener<T> implements ActionListener {

	private ListItem<T> listItem;
	
	public ListItemExpandListener(ListItem<T> listItem) {
		this.listItem = listItem;
	}
	
	@Override
	public void actionPerformed(ActionEvent e) {
		if(e.getActionCommand().equals("expand")) listItem.setExpand(true);
		else if(e.getActionCommand().equals("compress")) listItem.setExpand(false);
		listItem.requestUpdate();
	}
}
