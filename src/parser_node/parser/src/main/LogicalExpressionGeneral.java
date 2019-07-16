package main;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;

public class LogicalExpressionGeneral implements LogicalExpression {
	
	private LogicalExpressionGeneral[] expressions;
	private final String name;
	private String mappedOnValue;
	private HashSet<LogicalExpressionEmpty> noMappings = new HashSet<>();
	
	public LogicalExpressionGeneral(String name, int arguments) {
		this.name = name;
		expressions = new LogicalExpressionGeneral[arguments];
	}
	
	public LogicalExpressionGeneral(String name, LogicalExpressionGeneral[] children) {
		this.name = name;
		this.expressions = children;
	}

	public String getName() {
		return this.name;
	}
	
	public HashMap<String, Float> getFeatures(){
		HashMap<String, Float> features = new HashMap<>();
		return getFeatures(features);
	}
	
	public HashMap<String, Float> getFeatures(HashMap<String, Float> features){
		//Feature for mapping
		if(mappedOnValue != null) {
			String map_feature = mappedOnValue + " -> " + name;
			features.put(map_feature, features.getOrDefault(map_feature, (float) 0) + 1);			
		}
		//Feature for inside
		String second_feature = getName() + " incl. ";
		for(LogicalExpressionGeneral expr : expressions) {
			String new_feature;
			if(expr == null)
				new_feature = second_feature + "null";
			else
				new_feature = second_feature + expr.getName();
			features.put(new_feature, features.getOrDefault(new_feature, (float) 0) + 1);
		}
		//Features of children
		for(LogicalExpression expr : expressions) {
			if(expr != null)
				expr.getFeatures(features);
		}
		//For empty mappings
		for(LogicalExpressionEmpty em : noMappings) {
			features = em.getFeatures(features);
		}
		return features;
	}
	
	@Override
	public String toString() {
		String str = this.name + "(";
		for(LogicalExpression expr :expressions) {
			if(expr != null)
				str += expr.toString() + ", ";
			else
				str += "null" + ", ";
		}
		str = str.substring(0, str.length()-2);
		str += ")";
		return str;
	}
	
	public void mapOn(String word) {
		this.mappedOnValue = word;
	}
	
	@Override
	public boolean equals(Object o) {
		if(o == null) return false;
		if(!(o instanceof LogicalExpressionGeneral)) return false;
		LogicalExpressionGeneral l = (LogicalExpressionGeneral) o;
		if(!this.toString().equals(o.toString())) return false;
		if(this.size() !=  l.size()) return false;
		if(!this.name.equals(l.name)) return false;
		if(!this.mappedOnValue.equals(l.mappedOnValue)) return false;
		HashSet<LogicalExpressionEmpty> emptyMappings = l.emptyMappings();
		for(LogicalExpressionEmpty empty : this.emptyMappings()) {
			if(!emptyMappings.contains(empty)) return false;
		}
		for(int i = 0; i < this.size(); i++) {
			if(expressions[i] == null && l.expressions[i] != null) return false;
			if(expressions[i] == null) continue;
			if(!expressions[i].equals(l.expressions[i])) return false;
		}
		return true;		
	}
	
	@Override
	public int hashCode() {
		return this.toString().hashCode();
	}
	
	public LogicalExpressionGeneral deepCopy() {
		LogicalExpressionGeneral[] children = new LogicalExpressionGeneral[expressions.length];
		for(int i = 0; i < children.length; i++) {
			if(expressions[i] == null)
				children[i] = null;
			else
				children[i] = expressions[i].deepCopy();
		}
		LogicalExpressionGeneral expr = new LogicalExpressionGeneral(this.name, children);
		if(this.mappedOnValue != null)
			expr.mappedOnValue = this.mappedOnValue;
		for(LogicalExpressionEmpty emp : this.noMappings) {
			expr.addEmptyMapping(emp.deepCopy());
		}
		return expr;
	}
	
	@Override
	public String callFormat() {
		String str = name.substring(0, 1).toLowerCase() + name.substring(1) + "(";
		for(LogicalExpression expr :expressions) {
			if(expr != null)
				str += expr.callFormat() + ", ";
			else
				str += "_" + ", ";
		}
		str = str.substring(0, str.length()-2);
		str += ")";
		return str;
	}
	
	public void addEmptyMapping(Collection<LogicalExpressionEmpty> empty) {
		noMappings.addAll(empty);
	}
	
	public void addEmptyMapping(LogicalExpressionEmpty empty) {
		noMappings.add(empty);
	}

	@Override
	public boolean free() {
		return this.expressions[0] == null;
	}

	public void addExpressions(LogicalExpressionGeneral[] expression) {
		this.expressions = expression;
	}
	
	public int size() {
		return this.expressions.length;
	}
	
	public HashSet<LogicalExpressionEmpty> emptyMappings(){
		HashSet<LogicalExpressionEmpty> empty = new HashSet<>(this.noMappings);
		for(LogicalExpressionGeneral exp : expressions) {
			if(exp != null)
				empty.addAll(exp.emptyMappings());
		}
		return empty;
	}

	@Override
	public HashMap<String, Integer> components() {
		return null;
	}

}
