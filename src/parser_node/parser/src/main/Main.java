package main;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import java.util.stream.Collectors;

import sgd.SGD;
import sgd.TrainingExample;
import sgd_implementation.SGD_Helper_Implementation;

public class Main {

	public static void main(String[] args) throws IOException, ClassNotFoundException{

		SGD_Helper_Implementation helper = new SGD_Helper_Implementation();

		/*SGD<String, String, LogicalExpression> sgd = new SGD<>(helper);

		List<TrainingExample<String, LogicalExpression>> examples = parseDataset();

		for(TrainingExample<String, LogicalExpression> example : examples) {
			System.out.println(example.getInputData().toString() + "  =>  " + example.getCorrectOutput().toString());
		}

		Collections.shuffle(examples, new Random(System.nanoTime()));
		examples = examples.stream().sorted((TrainingExample<String, LogicalExpression> t1, TrainingExample<String, LogicalExpression> t2) -> Integer.compare(t1.getInputData().length(), t2.getInputData().length())).collect(Collectors.toList());
		HashMap<String, Float> ws = sgd.run(examples, 10, (float) 0.1);
		System.out.println(ws);
		saveWeights(ws, "part_1-3");
		System.err.println("DONE");*/

		HashMap<String, Float> w = loadWeights("C:\\Users\\Shani\\InternshipDTAI\\codePJ\\parsing\\trained_vectors\\trained_fold_0");
		System.out.println(w);
		System.err.println("LOADED");
		/*Scanner input = new Scanner(System.in);
		String sentence = input.nextLine();
		while(sentence.length() > 0) {
			String result = helper.parse(sentence, w);
			System.out.println(result);
			sentence = input.nextLine();			
		}
		input.close();*/

		ArrayList<String> parseroutput = new ArrayList<>();
		ArrayList<String> speechinput = readFromSpeech();
		for(String str : speechinput){
			String expression = helper.parse(str, w);
			parseroutput.add(expression);
		}
		writeToFile(parseroutput);
	}

	private static ArrayList<String> readFromSpeech() throws FileNotFoundException {
		ArrayList<String> speech = new ArrayList<>();
		File file = new File("C:\\Users\\Shani\\InternshipDTAI\\common\\speechinput.txt");
		Scanner sc = new Scanner(file);
		while(sc.hasNextLine()){
			String line = sc.nextLine();
			if(line.length() == 0)
				break;
			speech.add(line);
		}
		sc.close();
		return speech;
	}

	private static void writeToFile(ArrayList<String> logicalExpressions) throws IOException {
		File file = new File("C:\\Users\\Shani\\InternshipDTAI\\common\\parseroutput.txt");
		FileOutputStream fos = new FileOutputStream(file, true);
		for(String str : logicalExpressions){
			fos.write(str.getBytes());
			fos.write(System.lineSeparator().getBytes());
		}
		fos.close();
	}

	//Reads in a dataset of training examples!
	private static ArrayList<TrainingExample<String, LogicalExpression>> parseDataset() throws FileNotFoundException {
		ArrayList<TrainingExample<String, LogicalExpression>> examples = new ArrayList<>();

		//This points to one of the folds in the dataset folder!!
		File file = new File("C:\\Users\\Shani\\InternshipDTAI\\codePJ\\parsing\\dataset\\fold_0");
		Scanner sc = new Scanner(file);
		while (sc.hasNextLine()) {
			String line = sc.nextLine();
			if (line.length() == 0)
				break;
			String[] data = line.split(";");
			examples.add(new TrainingExample<String, LogicalExpression>(data[1].trim(),
					new LogicalExpressionTraining(data[2].trim())));
		}
		return examples;
	}

	private static ArrayList<TrainingExample<String, LogicalExpression>> getExamples() {
		ArrayList<TrainingExample<String, LogicalExpression>> examples = new ArrayList<>();
		examples.add(new TrainingExample<String, LogicalExpression>("put the red block on the green block",
				new LogicalExpressionTraining("Put(On(Red(Block(null)), Green(Block(null))))")));
		examples.add(new TrainingExample<String, LogicalExpression>("put the blue block on the green block",
				new LogicalExpressionTraining("Put(On(Blue(Block(null)), Green(Block(null))))")));
		examples.add(new TrainingExample<String, LogicalExpression>("put the blue block on the red block",
				new LogicalExpressionTraining("Put(On(Blue(Block(null)), Red(Block(null))))")));
		examples.add(new TrainingExample<String, LogicalExpression>("put the blue block on the blue block",
				new LogicalExpressionTraining("Put(On(Blue(Block(null)), Blue(Block(null))))")));
		return examples;
	}

	private static void saveWeights(HashMap<String, Float> w, String fileName) throws IOException {
		FileOutputStream myFileOutputStream = new FileOutputStream(fileName);
		ObjectOutputStream myObjectOutputStream = new ObjectOutputStream(myFileOutputStream);
		myObjectOutputStream.writeObject(w);
		myObjectOutputStream.close();
	}

	private static HashMap<String, Float> loadWeights(String fileName) throws IOException, ClassNotFoundException {
		FileInputStream myFileInputStream = new FileInputStream(fileName);
		ObjectInputStream myObjectInputStream = new ObjectInputStream(myFileInputStream);
		HashMap<String, Float> done = (HashMap<String, Float>) myObjectInputStream.readObject();
		myObjectInputStream.close();
		return done;
	}
}
