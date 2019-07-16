package test;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;
import java.util.Scanner;

public class SplitDataset {

	public static void main(String[] args) throws IOException {
		ArrayList<ArrayList<String>> splitted = new ArrayList<>();
		for(int i = 0; i < 10; i++) splitted.add(new ArrayList<>());
		File file = new File("/home/pieter/Documenten/masterProef/python_code/dataset");
		Scanner sc = new Scanner(file);
		Random r = new Random();
		while (sc.hasNextLine()) {
			String line = sc.nextLine();
			if (line.length() == 0) continue;
			int i = r.nextInt(10);
			while(splitted.get(i).size() >= 10) i = r.nextInt(10);
			splitted.get(i).add(line);
		}
		sc.close();
		writeToFiles(splitted);
	}
	
	public static void writeToFiles(ArrayList<ArrayList<String>> splitted) throws IOException {
		for(int i = 0; i < 10; i++) {
			ArrayList<String> values = splitted.get(i);
			String file_name = "fold_" + Integer.toString(i);
			BufferedWriter writer = new BufferedWriter(
	                new FileWriter("dataset/" + file_name)
	            ); 
			int count = 0;
			for(String line : values) {
				if(count > 0) writer.newLine();   //Add new line
				count++;
				writer.write(line);
			}
			writer.close();
		}
	}

}
