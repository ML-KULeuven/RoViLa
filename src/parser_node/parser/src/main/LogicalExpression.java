package main;

import java.util.HashMap;

public interface LogicalExpression {
	
	public LogicalExpression deepCopy();
	
	public HashMap<String, Float> getFeatures();
	
	public boolean free();
	
	public HashMap<String, Integer> components();
	
	public String getName();

	HashMap<String, Float> getFeatures(HashMap<String, Float> features);

	public String callFormat();

}
