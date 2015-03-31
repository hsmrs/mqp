package src.main.java.com.github.hsmrs_gui.project.view.list;

import java.awt.Point;
import java.util.List;

public interface ListItemListener<T> {
	public void itemsSelected(List<T> listObjects); // Or deselected
	public void itemDoubleClicked(T listObject);
	public void itemRightClicked(T listObject, Point p);
	public void itemFocused(T listObject);
}
