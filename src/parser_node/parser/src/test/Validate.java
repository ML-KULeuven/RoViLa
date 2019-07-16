package test;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;

import main.LogicalExpression;
import main.LogicalExpressionTraining;
import sgd.TrainingExample;
import sgd.Tuple;
import sgd_implementation.SGD_Helper_Implementation;

public class Validate {
	
	private static final String index = "1";
	
	public static void main(String[] args) throws IOException, ClassNotFoundException{
		SGD_Helper_Implementation helper = new SGD_Helper_Implementation();
		HashMap<String, Float> w = loadWeights();
		System.out.println(w);
		int correct = 0;
		for(Tuple<String,TrainingExample<String, LogicalExpression>> data : testSet()) {
			System.out.println(data.gety().getInputData());
			String result = helper.parseContext(data.getx(), data.gety().getInputData(), w);
			//System.out.println(data.gety().getInputData());
			//String result = helper.parse(data.gety().getInputData(), w);
			if(result.equals(data.gety().getCorrectOutput().toString())) {
				correct++;
			} else {
				System.out.println(data.gety().getInputData() + "  =>  " + result);
			}
		}
		System.out.println("Accuracy = " + Double.toString(((double) correct) / 10));
	}
	
	private static HashMap<String, Float> loadWeights() throws IOException, ClassNotFoundException {
		FileInputStream myFileInputStream = new FileInputStream("result/results2/trained_fold_" + index);
		ObjectInputStream myObjectInputStream = new ObjectInputStream(myFileInputStream);
		HashMap<String, Float> done = (HashMap<String, Float>) myObjectInputStream.readObject();
		myObjectInputStream.close();
		return done;
	}
	
	private static ArrayList<Tuple<String,TrainingExample<String, LogicalExpression>>> testSet() throws FileNotFoundException {
		ArrayList<Tuple<String,TrainingExample<String, LogicalExpression>>> examples = new ArrayList<>();
		File file = new File("dataset/fold_" + index);
		Scanner sc = new Scanner(file);
		while (sc.hasNextLine()) {
			String line = sc.nextLine();
			if (line.length() == 0)
				break;
			String[] data = line.split(";");
			examples.add(new Tuple<String, TrainingExample<String, LogicalExpression>>(data[0].trim(), new TrainingExample<String, LogicalExpression>(data[1].trim(),
					new LogicalExpressionTraining(data[2].trim()))));
		}
		sc.close();
		return examples;
	}

}
