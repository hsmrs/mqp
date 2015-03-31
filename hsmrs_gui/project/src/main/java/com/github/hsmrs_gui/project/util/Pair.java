package src.main.java.com.github.hsmrs_gui.project.util;

public class Pair <A, B>{
	public final A X;
	public final B Y;
	
	public Pair(A x, B y){
		X = x;
		Y = y;
	}
	
	/**
	 * Determines if this Pair is equal to another Pair.
	 * @param other The other Pair
	 * @return True if these Pair's are equal.
	 */
	public boolean equalTo(Pair other){
		return X == other.X && Y == other.Y;
	}
	
	/**
	 * Returns a string representation of the Pair.
	 */
	public String toString(){
		return "(" + X + ", " + Y + ")";
	}
}
