package main;

import java.util.HashMap;

public class LogicalExpressionEmpty implements LogicalExpression{
	
	private String mapped_on_value;
	
	public LogicalExpressionEmpty(String mapped_on) {
		this.mapped_on_value = mapped_on;
	}
	
	public HashMap<String, Float> getFeatures(){
		HashMap<String, Float> features = new HashMap<>();
		return getFeatures(features);
	}
	
	public HashMap<String, Float> getFeatures(HashMap<String, Float> features){
		String feature = mapped_on_value + " -> null";
		features.put(feature, features.getOrDefault(feature, (float) 0)+1);
		return features;
	}
	
	public LogicalExpressionEmpty deepCopy() {
		return new LogicalExpressionEmpty(this.mapped_on_value);
	}

	@Override
	public boolean free() {
		return false;
	}
	

	@Override
	public String toString() {
		return "empty";
	}
	
	@Override
	public boolean equals(Object o) {
		if(!(o instanceof LogicalExpressionEmpty)) return false;
		LogicalExpressionEmpty e = (LogicalExpressionEmpty) o;
		return e.mapped_on_value.equals(this.mapped_on_value);
	}
	
	@Override
	public int hashCode() {
		return this.mapped_on_value.hashCode();
	}

	@Override
	public HashMap<String, Integer> components() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String getName() {
		return null;
	}

	@Override
	public String callFormat() {
		// TODO Auto-generated method stub
		return null;
	}
}
