package test;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map.Entry;

import main.LogicalExpression;
import main.LogicalExpressionTraining;
import sgd_implementation.SGD_Helper_Implementation;

public class MainTest {

	public static void main(String[] args) {
		LogicalExpressionTraining exp = new LogicalExpressionTraining("Put(On(Red(Block(null)), Green(Block(null))))");
		System.out.println(exp.components());
		System.exit(0);
		/**SGD_Helper_Implementation helper = new SGD_Helper_Implementation();
		System.out.println("=====");
		Collection<LogicalExpression> result2 = helper.getPossibleOutputs("hello word", new HashMap<>());
		for(LogicalExpression exp : result2) {
			System.out.println(exp);
			//printHashmap(exp.getFeatures());
			System.out.println("-------------");
		}**/
	}
	
	public static void printHashmap(HashMap<String, Integer> map) {
		String str = "{";
		for(Entry<String, Integer> entry : map.entrySet()) {
			str += entry.getKey() + ": " + Float.toString(entry.getValue()) + ", ";
		}
		System.out.println(str.substring(0, str.length()-2) + "}");
	}

}
