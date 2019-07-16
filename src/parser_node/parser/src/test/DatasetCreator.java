package test;

import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.HashMap;
import java.util.Scanner;

import main.LogicalExpressionTraining;
import sgd_implementation.SGD_Helper_Implementation;

public class DatasetCreator {
	
	private static final String path = "/home/pieter/Documenten/masterProef/python_code/dataset";
	
	public static void main(String[] args) throws ClassNotFoundException, IOException {
		SGD_Helper_Implementation helper = new SGD_Helper_Implementation();
		HashMap<String, Float> w = loadWeights("part_1-3");
		System.out.println(w);
		System.err.println("LOADED");
		Scanner input = new Scanner(System.in);
		System.out.print("Give image number:");
		String number = input.nextLine();
		System.out.println("Enter sentences");
		String sentence = input.nextLine();
		while(sentence.length() > 0) {
			String result = helper.parse(sentence, w);
			addToData(number + "; " + sentence + "; " + result);
			System.out.println(result);
			sentence = input.nextLine();			
		}
		input.close();
		/**SGD_Helper_Implementation helper = new SGD_Helper_Implementation();
		System.out.println("=====");
		Collection<LogicalExpression> result2 = helper.getPossibleOutputs("hello word", new HashMap<>());
		for(LogicalExpression exp : result2) {
			System.out.println(exp);
			//printHashmap(exp.getFeatures());
			System.out.println("-------------");
		}**/
	}
	
	private static void addToData(String str) throws IOException {
		BufferedWriter writer = new BufferedWriter(
                new FileWriter(path, true)  //Set true for append mode
            ); 
		writer.newLine();   //Add new line
		writer.write(str);
		writer.close();
	}
	
	private static HashMap<String, Float> loadWeights(String fileName) throws IOException, ClassNotFoundException {
		FileInputStream myFileInputStream = new FileInputStream(fileName);
		ObjectInputStream myObjectInputStream = new ObjectInputStream(myFileInputStream);
		HashMap<String, Float> done = (HashMap<String, Float>) myObjectInputStream.readObject();
		myObjectInputStream.close();
		return done;
	}

}
