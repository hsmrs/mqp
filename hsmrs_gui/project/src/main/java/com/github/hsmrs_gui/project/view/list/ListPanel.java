package src.main.java.com.github.hsmrs_gui.project.view.list;

import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.Scrollable;

import net.miginfocom.swing.MigLayout;

public class ListPanel extends JPanel implements Scrollable {
	
	private boolean tracksViewportHeight;
	private boolean tracksViewportWidth;
	
	private List<Component> components;
	
	public ListPanel() {
		this.components = new ArrayList<Component>();
		this.setLayout(new MigLayout("fill, insets 0, gap 0 0 0 0, flowy"));
		this.setBackground(Color.white);
	}
	
	public void setTracksViewportHeight(boolean tracksViewportHeight) {
		this.tracksViewportHeight = tracksViewportHeight;
	}
	
	public void setTracksViewportWidth(boolean tracksViewportWidth) {
		this.tracksViewportWidth = tracksViewportWidth;
	}
	
	public int getIndexOfComponent(Component c) {
		return components.indexOf(c);
	}
	
	@Override
	public Component add(Component c) {
		components.add(c);
		this.add(c, "pushx, growx");
		return c;
	}
	
	@Override
	public boolean getScrollableTracksViewportHeight() {
		return tracksViewportHeight;
	}

	@Override
	public boolean getScrollableTracksViewportWidth() {
		return tracksViewportWidth;
	}
	
	@Override
	public Dimension getPreferredScrollableViewportSize() {
		return this.getPreferredSize();
	}

	@Override
	public int getScrollableBlockIncrement(Rectangle visibleRect,
			int orientation, int direction) {
		return 32;
	}

	@Override
	public int getScrollableUnitIncrement(Rectangle visibleRect,
			int orientation, int direction) {
		return 32;
	}

	public void clear() {
		components.clear();
		this.removeAll();
	}
}
