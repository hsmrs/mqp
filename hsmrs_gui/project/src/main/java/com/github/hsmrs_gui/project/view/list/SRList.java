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

import src.main.java.com.github.hsmrs_gui.project.model.RobotModel;

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

	public void setListItemRenderer(ListItemRenderer<T> renderer) {
		this.listItemRenderer = renderer;
		GuiNode.getLog().info("Set List Item Renderer");
	}

	@Override
	public void setHorizontalScrollBarPolicy(int policy) {
		if(listPanel != null) {
			listPanel.setTracksViewportWidth(policy == JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
			super.setHorizontalScrollBarPolicy(policy);
		}
	}

	@Override
	public void setVerticalScrollBarPolicy(int policy) {
		if(listPanel != null) {
			listPanel.setTracksViewportHeight(policy == JScrollPane.VERTICAL_SCROLLBAR_NEVER);
			super.setVerticalScrollBarPolicy(policy);
		}
	}

	// Logic / Data Builders / Other
	private void fillListItems() {
		listItems.clear();
		selectedListItems.clear();
		for(int i = 0; i < listModel.getSize(); i++) listItems.add(new ListItem<T>(this, (T)listModel.getElementAt(i)));
	}

	private void createComponents() {
		listPanel.clear();
		listItemRenderer.createRenderedListComponents(this, listItems);
		for(ListItem<T> listItem: listItems) {
			listPanel.add(listItem.getComponent());
		}
		this.revalidate();
		this.repaint();
	}

	public void updateComponents() {
		listItemRenderer.updateRenderedListComponents(this, listItems);
		this.revalidate();
		this.repaint();
	}

	private void clearSelection() {
		for(ListItem<T> listItem: selectedListItems) {
			listItem.setSelected(false);
			listItem.setFocused(false);
		}
		this.selectedListItems.clear();
	}

	public List<T> getSelectedItems() {
		List<T> selected = new ArrayList<T>();
		for(ListItem<T> listItem: selectedListItems) selected.add(listItem.getListObject());
		return selected;
	}

	// Listeners
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

	@Override
	public void mouseReleased(MouseEvent e) {
		if (e.isPopupTrigger()) {
			int indexOfMousePress = listPanel.getIndexOfComponent(listPanel.getComponentAt(e.getPoint()));
			if(indexOfMousePress >= 0) {
				this.fireItemRightClicked(listItems.get(indexOfMousePress).getListObject(), e.getPoint());
			}
		}
	}

	@Override
	public void contentsChanged(ListDataEvent e) {
		createComponents();
	}

	@Override
	public void intervalAdded(ListDataEvent e) {
		fillListItems();
		createComponents();
	}

	@Override
	public void intervalRemoved(ListDataEvent e) {
		fillListItems();
		createComponents();
	}

	public void fireItemDoubleClicked(T listObject) {
		for(ListItemListener<T> l: listItemListeners) l.itemDoubleClicked(listObject);
	}

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
