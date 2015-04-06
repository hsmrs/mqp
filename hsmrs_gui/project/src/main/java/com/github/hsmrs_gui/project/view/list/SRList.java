package src.main.java.com.github.hsmrs_gui.project.view.list;

import java.awt.Point;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JScrollPane;
import javax.swing.ListModel;
import javax.swing.SwingUtilities;
import javax.swing.event.ListDataEvent;
import javax.swing.event.ListDataListener;

import org.apache.commons.logging.Log;

import src.main.java.com.github.hsmrs_gui.project.model.robot.RobotModel;

import com.github.hsmrs_gui.project.GuiNode;


public class SRList<T> extends JScrollPane implements MouseListener, ListDataListener {
	// Variables
	private ListItemRenderer<T> listItemRenderer;
	private ListModel listModel;
	private List<ListItem<T>> listItems;
	private ListPanel listPanel;

	private List<ListItem<T>> selectedListItems;

	private int selectionStartIndex;

	List<ListItemListener<T>> listItemListeners;

	/**
	 * The constructor for the SRList, which is a vertical list of renderable list item components.
	 * @param listModel The model which contains the items in the list.
	 * @param listItemRenderer The rendering engine which renders the ListItems.
	 */
	public SRList(ListModel listModel, ListItemRenderer<T> listItemRenderer) {
		this.listModel = listModel;
		listItems = new ArrayList<ListItem<T>>();
		listItemListeners = new ArrayList<ListItemListener<T>>();
		selectedListItems = new ArrayList<ListItem<T>>();

		listPanel = new ListPanel();
		listPanel.addMouseListener(this);
		this.addMouseListener(this);
		this.listItemRenderer = listItemRenderer;

		this.setViewportView(listPanel);
		this.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
		this.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED);

		fillListItems();
		this.listModel.addListDataListener(this);
		createComponents();
	}
	
	// ListPanel Settings
	/**
	 * Sets the rendering engine for ListItems.
	 * @param renderer The new rendering engine.
	 */
	public void setListItemRenderer(ListItemRenderer<T> renderer) {
		this.listItemRenderer = renderer;
		GuiNode.getLog().info("Set List Item Renderer");
	}

	/**
	 * Sets whether or not the list can scroll horizontally or not.
	 * @param policy Either JScrollPane.HORIZONTAL_SCROLLBAR_[ALWAYS, AS_NEEDED, NEVER]. 
	 */
	@Override
	public void setHorizontalScrollBarPolicy(int policy) {
		if(listPanel != null) {
			listPanel.setTracksViewportWidth(policy == JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
			super.setHorizontalScrollBarPolicy(policy);
		}
	}

	/**
	 * Sets whether or not the list can scroll vertically or not.
	 * @param policy Either JScrollPane.VERTICAL_SCROLLBAR_[ALWAYS, AS_NEEDED, NEVER]. 
	 */
	@Override
	public void setVerticalScrollBarPolicy(int policy) {
		if(listPanel != null) {
			listPanel.setTracksViewportHeight(policy == JScrollPane.VERTICAL_SCROLLBAR_NEVER);
			super.setVerticalScrollBarPolicy(policy);
		}
	}

	// Logic / Data Builders / Other
	/**
	 * This method pulls all of the items from the ListModel and adds them as ListItems to the SRList.
	 */
	private void fillListItems() {
		listItems.clear();
		selectedListItems.clear();
		for(int i = 0; i < listModel.getSize(); i++) listItems.add(new ListItem<T>(this, (T)listModel.getElementAt(i)));
	}

	/**
	 * Use the rendering engine to turn the ListItems into AWT Components.
	 */
	private void createComponents() {
		listPanel.clear();
		listItemRenderer.createRenderedListComponents(this, listItems);
		for(ListItem<T> listItem: listItems) {
			listPanel.add(listItem.getComponent());
		}
		this.revalidate();
		this.repaint();
	}

	/**
	 * Use the rendering engine to update the ListItem AWT Components.
	 */
	public void updateComponents() {
		listItemRenderer.updateRenderedListComponents(this, listItems);
		this.revalidate();
		this.repaint();
	}

	/**
	 * Clears all of the selected ListItems.
	 */
	private void clearSelection() {
		for(ListItem<T> listItem: selectedListItems) {
			listItem.setSelected(false);
			listItem.setFocused(false);
		}
		this.selectedListItems.clear();
	}

	/**
	 * Gets a list of the selected ListItems' underlying objects.
	 * @return A list of the selected ListItems' underlying objects.
	 */
	public List<T> getSelectedItems() {
		List<T> selected = new ArrayList<T>();
		for(ListItem<T> listItem: selectedListItems) selected.add(listItem.getListObject());
		return selected;
	}

	// Listeners
	/**
	 * This method is called whenever the mouse is pressed within the SRList.
	 */
	@Override
	public void mousePressed(MouseEvent e) {
		int indexOfMousePress = listPanel.getIndexOfComponent(listPanel.getComponentAt(e.getPoint()));
		
		if (e.isPopupTrigger()) {
			if(indexOfMousePress >= 0) {
				this.fireItemRightClicked(listItems.get(indexOfMousePress).getListObject(), e.getPoint());
			}
			return;
		}
		
		if(indexOfMousePress >= 0) {
			if(!SwingUtilities.isRightMouseButton(e)) {
				if(e.getClickCount() == 2) {
					clearSelection();
					listItems.get(indexOfMousePress).toggleDoubleClicked();
					this.fireItemDoubleClicked(listItems.get(indexOfMousePress).getListObject());
				} else if(e.isShiftDown()) {
					clearSelection();
					if(selectionStartIndex <= indexOfMousePress) {
						for(int i = selectionStartIndex; i <= indexOfMousePress; i++) {
							listItems.get(i).setSelected(true);
							selectedListItems.add(listItems.get(i));
						}
					} else if(selectionStartIndex > indexOfMousePress) {
						for(int i = indexOfMousePress; i <= selectionStartIndex; i++) {
							listItems.get(i).setSelected(true);
							selectedListItems.add(listItems.get(i));
						}
					}
					this.fireItemsSelected(getSelectedItems());
				} else if(e.isControlDown()) {
					listItems.get(selectionStartIndex).setFocused(false);
					if(listItems.get(indexOfMousePress).isSelected()) {
						listItems.get(indexOfMousePress).setSelected(false);
						listItems.get(indexOfMousePress).setFocused(false);
						selectedListItems.remove(listItems.get(indexOfMousePress));
					} else {
						listItems.get(indexOfMousePress).setSelected(true);
						listItems.get(indexOfMousePress).setFocused(true);
						selectedListItems.add(listItems.get(indexOfMousePress));
					}
					selectionStartIndex = indexOfMousePress;
					this.fireItemsSelected(getSelectedItems());
				} else {
					// Clear the selection
					boolean prevSelectionState = listItems.get(indexOfMousePress).isSelected();
					clearSelection();
					if(prevSelectionState) {
						listItems.get(indexOfMousePress).setSelected(false);
						listItems.get(indexOfMousePress).setFocused(false);
						selectedListItems.remove(listItems.get(indexOfMousePress));
					} else {
						listItems.get(indexOfMousePress).setSelected(true);
						listItems.get(indexOfMousePress).setFocused(true);
						selectedListItems.add(listItems.get(indexOfMousePress));
					}
					selectionStartIndex = indexOfMousePress;
					this.fireItemsSelected(getSelectedItems());
				}
			} else {
				if(!listItems.get(indexOfMousePress).isSelected()) {
					listItems.get(indexOfMousePress).setSelected(true);
					listItems.get(indexOfMousePress).setFocused(true);
					selectedListItems.add(listItems.get(indexOfMousePress));
				}
				selectionStartIndex = indexOfMousePress;
			}
		} else {
			selectionStartIndex = 0;
			clearSelection();
		}
		updateComponents();
	}

	/**
	 * This method is called whenever the mouse is released within the SRList.
	 */
	@Override
	public void mouseReleased(MouseEvent e) {
		if (e.isPopupTrigger()) {
			int indexOfMousePress = listPanel.getIndexOfComponent(listPanel.getComponentAt(e.getPoint()));
			if(indexOfMousePress >= 0) {
				this.fireItemRightClicked(listItems.get(indexOfMousePress).getListObject(), e.getPoint());
			}
		}
	}

	/**
	 * This method is called when the contents of the ListModel is changed. It recreates all of the SRList components.
	 */
	@Override
	public void contentsChanged(ListDataEvent e) {
		createComponents();
	}

	/**
	 * This method is called when items are added to the ListModel. It recreates the components in the SRList.
	 */
	@Override
	public void intervalAdded(ListDataEvent e) {
		fillListItems();
		createComponents();
	}

	/**
	 * This method is called when items are removed from the ListModel. It recreates the components in the SRList.
	 */
	@Override
	public void intervalRemoved(ListDataEvent e) {
		fillListItems();
		createComponents();
	}

	/**
	 * This method is called when an item is double clicked.
	 * @param listObject The object contained within the ListItem that was double clicked.
	 */
	public void fireItemDoubleClicked(T listObject) {
		for(ListItemListener<T> l: listItemListeners) l.itemDoubleClicked(listObject);
	}

	/**
	 * This method is called when an item is right clicked.
	 * @param listObject The underlying object of the right clicked list item.
	 * @param p The location of the right click.
	 */
	public void fireItemRightClicked(T listObject, Point p) {
		for(ListItemListener<T> l: listItemListeners) l.itemRightClicked(listObject, p);
	}

	public void fireItemsSelected(List<T> listObjects) {
		for(ListItemListener<T> l: listItemListeners) l.itemsSelected(listObjects);
	}

	public void fireItemFocused(T listObject) {
		for(ListItemListener<T> l: listItemListeners) l.itemFocused(listObject);
	}

	public void addListItemListener(ListItemListener<T> l) {
		listItemListeners.add(l);
	}

	public void removeListItemListener(ListItemListener<T> l) {
		listItemListeners.remove(l);
	}

	public List<ListItemListener<T>> getListItemListeners() {
		return listItemListeners;
	}

	// Unused
	@Override
	public void mouseClicked(MouseEvent e) {}
	@Override
	public void mouseEntered(MouseEvent e) {}
	@Override
	public void mouseExited(MouseEvent e) {}
}
