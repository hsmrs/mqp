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
	
	/**
	 * The constructor for the ListPanel class.
	 */
	public ListPanel() {
		this.components = new ArrayList<Component>();
		this.setLayout(new MigLayout("fill, insets 0, gap 0 0 0 0, flowy"));
		this.setBackground(Color.white);
	}
	
	/**
	 * This function sets whether or not the list panel scrolls to the end of the list.
	 * @param tracksViewportHeight True if the ListPanel tracks to the bottom of the list.
	 */
	public void setTracksViewportHeight(boolean tracksViewportHeight) {
		this.tracksViewportHeight = tracksViewportHeight;
	}
	
	/**
	 * This function sets whether or not the list panel scrolls to the right most side of the panel.
	 * @param tracksViewportWidth True if the list panel scrolls to the right most side of the panel.
	 */
	public void setTracksViewportWidth(boolean tracksViewportWidth) {
		this.tracksViewportWidth = tracksViewportWidth;
	}
	
	/**
	 * Get the index of the given AWT Component in the list.
	 * @param c The Component whose index is desired.
	 * @return The index of the Component.
	 */
	public int getIndexOfComponent(Component c) {
		return components.indexOf(c);
	}
	
	/**
	 * Add an AWT Component to the ListPanel.
	 * @param c The Component to add to the list panel.
	 */
	@Override
	public Component add(Component c) {
		components.add(c);
		this.add(c, "pushx, growx");
		return c;
	}
	
	/**
	 * Get the height of the listpanel visible area.
	 * @return The height of the listpanel visible area.
	 */
	@Override
	public boolean getScrollableTracksViewportHeight() {
		return tracksViewportHeight;
	}

	/**
	 * Get the width of the listpanel visible area.
	 * @return The width of the listpanel visible area.
	 */
	@Override
	public boolean getScrollableTracksViewportWidth() {
		return tracksViewportWidth;
	}
	
	/**
	 * Get the preferred Dimension of the listpanel visible area.
	 * @return The preferred Dimension of the listpanel visible area.
	 */
	@Override
	public Dimension getPreferredScrollableViewportSize() {
		return this.getPreferredSize();
	}

	/**
	 * Gets the minimum number of pixels moved during a scroll.
	 * @return The minimum number of pixels moved during a scroll.
	 */
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
