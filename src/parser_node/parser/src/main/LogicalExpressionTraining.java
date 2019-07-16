package main;

import java.util.HashMap;

public class LogicalExpressionTraining implements LogicalExpression{
	
	private final String value;
	
	public LogicalExpressionTraining(String value) {
		this.value = value;
	}
	
	@Override
	public String toString() {
		return this.value;
	}

	@Override
	public LogicalExpression deepCopy() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public HashMap<String, Float> getFeatures() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean free() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public HashMap<String, Float> getFeatures(HashMap<String, Float> features) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public HashMap<String, Integer> components() {
		HashMap<String, Integer> components = new HashMap<>();
		String[] parts = value.replace(" ", "").replace(",", "").split("\\(|\\)");
		for(String part : parts) {
			if(part.equals("null") || part.length() == 0) continue;
			components.put(part, components.getOrDefault(part, 0)+1);
		}
		
		return components;
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
 