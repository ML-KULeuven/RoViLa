package sgd;

public class Tuple<I, O> implements Comparable<Tuple<I, O>>{
	
	public Tuple(I x, O y){
		this.setx(x);
		this.sety(y);
	}	
	
	public boolean equals(Tuple<I, O> tuple2){
		return tuple2.getx().equals(this.getx()) && tuple2.gety().equals(this.gety());
	}
	
	public I getx() {
		return x;
	}
	private void setx(I x) {
		this.x = x;
	}
	
	private I x;
	
	public O gety() {
		return y;
	}
	private void sety(O y) {
		this.y = y;
	}

	private O y;
	
	@Override
	public String toString() {
		return "Tuple(" + getx().toString() + ", " + gety().toString() + ")";
	}

	@Override
	public int compareTo(Tuple<I, O> o) {
		Comparable<I> v1 = (Comparable<I>) this.getx();
		int value =  -1 * (v1.compareTo(o.getx()));
		return value;
	}
	
	@Override
	public boolean equals(Object o) {
		try {
			Tuple<I, O> other = (Tuple<I, O>) o;
			return this.getx().equals(other.getx()) && this.gety().equals(other.gety());
		} catch(Exception e) {
			return false;
		}
	}
	
	
}
