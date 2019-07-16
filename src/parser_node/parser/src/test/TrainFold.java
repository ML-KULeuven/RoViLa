package test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import java.util.stream.Collectors;

import main.LogicalExpression;
import main.LogicalExpressionTraining;
import sgd.SGD;
import sgd.TrainingExample;
import sgd_implementation.SGD_Helper_Implementation;

public class TrainFold {
	
	public static void main(String[] args) throws IOException, ClassNotFoundException{
		SGD_Helper_Implementation helper = new SGD_Helper_Implementation();
		SGD<String, String, LogicalExpression> sgd = new SGD<>(helper);
		String index = args[0];
		List<TrainingExample<String, LogicalExpression>> examples = parseDataset(index);
		for(TrainingExample<String, LogicalExpression> example : examples) {
			System.out.println(example.getInputData().toString() + "  =>  " + example.getCorrectOutput().toString());
		}
		Collections.shuffle(examples, new Random(System.nanoTime()));
		examples = examples.stream().sorted((TrainingExample<String, LogicalExpression> t1, TrainingExample<String, LogicalExpression> t2) -> Integer.compare(t1.getInputData().split(" ").length, t2.getInputData().split(" ").length)).collect(Collectors.toList());
		HashMap<String, Float> w = sgd.run(examples, 10, (float) 0.1);
		System.out.println(w);
		saveWeights(w, "results/trained_fold_" + index);
		System.err.println("DONE");
	}

	private static ArrayList<TrainingExample<String, LogicalExpression>> parseDataset(String index) throws FileNotFoundException {
		ArrayList<TrainingExample<String, LogicalExpression>> examples = new ArrayList<>();
		for(int i = 0; i < 10; i++) {
			if(Integer.toString(i).equals(index.trim())) continue;
			File file = new File("dataset/fold_" + Integer.toString(i));
			Scanner sc = new Scanner(file);
			while (sc.hasNextLine()) {
				String line = sc.nextLine();
				if (line.length() == 0) continue;
				String[] data = line.split(";");
				examples.add(new TrainingExample<String, LogicalExpression>(data[1].trim(),
						new LogicalExpressionTraining(data[2].trim())));
			}
			sc.close();
		}
		return examples;
	}
	
	private static void saveWeights(HashMap<String, Float> w, String fileName) throws IOException {
		FileOutputStream myFileOutputStream = new FileOutputStream(fileName);
		ObjectOutputStream myObjectOutputStream = new ObjectOutputStream(myFileOutputStream);
		myObjectOutputStream.writeObject(w);
		myObjectOutputStream.close();
	}

}
