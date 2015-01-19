package src.main.java.com.github.hsmrs_gui.project.util;

public class Pair <A, B>{
	public final A X;
	public final B Y;
	
	public Pair(A x, B y){
		X = x;
		Y = y;
	}
	
	public String toString(){
		return "(" + X + ", " + Y + ")";
	}
}
