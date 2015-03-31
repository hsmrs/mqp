package src.main.java.com.github.hsmrs_gui.project.view.list;

import javax.swing.JComponent;

public class ListItem<T> {
	
	private SRList<T> srList;
	private T listObject;
	private boolean isSelected;
	private boolean isFocused;
	private boolean isDoubleClicked;
	private boolean isExpanded;
	private boolean isDirty;
	private JComponent component;
	private RenderableComponent<T> renderableComponent;
	
	/**
	 * The constructor for the ListItem class.
	 * @param srList The SRList which contains this ListItem.
	 * @param listItem The underlying object that this ListItem wraps.
	 */
	public ListItem(SRList<T> srList, T listItem) {
		this.srList = srList;
		this.listObject = listItem;
	}
	
	public T getListObject(){return this.listObject;}
	
	public JComponent getComponent(){return this.component;}
	public void setComponent(JComponent component) {this.component = component;}
	
	public RenderableComponent<T> getRenderableComponent(){return this.renderableComponent;}
	public void setRenderableComponent(RenderableComponent<T> renderableComponent) {this.renderableComponent = renderableComponent;}
	
	public boolean isFocused() {return isFocused;}
	public void setFocused(boolean focused) {this.isFocused = focused;}
	
	public boolean isSelected() {return this.isSelected;}
	public void setSelected(boolean selected) {this.isSelected = selected;}

	public boolean isDoubleClicked() {return this.isDoubleClicked;}
	public void setDoubleClicked(boolean doubleClicked){this.isDoubleClicked = doubleClicked;}

	public void toggleDoubleClicked() {this.isDoubleClicked = !this.isDoubleClicked;}

	public void setExpand(boolean expanded) {this.isExpanded = expanded;}
	public boolean isExpanded() {return this.isExpanded;}

	public boolean isDirty() {return this.isDirty;}
	
	public void requestUpdate() {
		this.isDirty = true;
		srList.updateComponents();
	}

	public void update() {
		this.renderableComponent.update(this);
		this.isDirty = false;
	}
}
