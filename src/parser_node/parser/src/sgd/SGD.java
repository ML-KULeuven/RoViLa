package sgd;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.Random;

import main.LogicalExpressionGeneral;

/**
* Generic Java Class for Stochastic Gradient Descent Algorithm
* 
* Needs as input a class that implements the interface Phi.java
* 
* To train the algorithm you need to input a list of Training Examples
*
* @author  Coenen Pieter-Jan
* @version 1.1
* @since   2016-10-16
*/

/**
 * 
 * @param <I> The type of input, probably String
 * @param <M> The type in which the algorithm needs to save the values for w
 * @param <O> The type of the output, probably String
 */
public class SGD<I, M, O> {

	/**
	 * 
	 * @param helper         A class that implements the the interface Phi.java,
	 *                       needed to calculate the score
	 * @param possibleValues An array which contains all possible values for the
	 *                       output
	 */
	public SGD(SGD_helper<I, M, O> helper) {
		this.setSGDHelper(helper);
	}

	/**
	 * 
	 * @param examples A list of TraingExamples which contain the data to train the
	 *                 algorithm
	 * @param T        An integer value that says how many times the algorithm needs
	 *                 to iterate
	 * @param eta      The value eta
	 * @return The function returns the variable W that contains a HashMap with the
	 *         values for the trained data which can be used for further training or
	 *         prediction
	 */
	public HashMap<M, Float> run(List<TrainingExample<I, O>> examples, int T, float eta) {
		HashMap<M, Float> w = this.getW();
		for (int i = 0; i < T; i++) {
			System.out.println("Start iteration " + Integer.toString(i));
			// random shuffle
			long seed = System.nanoTime();
			if(i > 0)
				Collections.shuffle(examples, new Random(seed));
			for (TrainingExample<I, O> example : examples) {
				Entry<Tuple<I, O>, Float> y_tilde = this.getMaxScore(w, example.getInputData(),
						example.getCorrectOutput());
				updateW(eta, w, example, y_tilde);
			}
		}
		return w;
	}

	private void updateW(float eta, HashMap<M, Float> w, TrainingExample<I, O> example,
			Entry<Tuple<I, O>, Float> y_tilde) {
		HashMap<M, Float> phi_y_tilde = helper.Phi(example.getInputData(), y_tilde.getKey().gety());
		O best_y = bestCorrectY(w, example);
		HashMap<M, Float> phi_y = helper.Phi(example.getInputData(), best_y);
		for (Entry<M, Float> entry : phi_y_tilde.entrySet()) {
			float actualValue = phi_y.getOrDefault(entry.getKey(), (float) 0);
			float newValue = eta * (actualValue - entry.getValue());
			w.put(entry.getKey(), w.getOrDefault(entry.getKey(), (float) 0) + newValue);
		}
		for (Entry<M, Float> entry : phi_y.entrySet()) {
			float predictedValue = phi_y_tilde.getOrDefault(entry.getKey(), (float) 0);
			float newValue = eta * (entry.getValue() - predictedValue);
			w.put(entry.getKey(), w.getOrDefault(entry.getKey(), (float) 0) + newValue);
		}
		System.out.println("Updated");
	}

	private O bestCorrectY(HashMap<M, Float> w, TrainingExample<I, O> example) {
		HashMap<Tuple<I, O>, Float> allScores = getAllScores(w, example.getInputData(), example.getCorrectOutput(), true);
		float max = 0;
		ArrayList<O> solutions = new ArrayList<>();
		for(Entry<Tuple<I, O>, Float> score : allScores.entrySet()) {
			if(score.getValue() == max) {
				solutions.add(score.getKey().gety());
			}else if(score.getValue() > max) {
				max = score.getValue();
				solutions = new ArrayList<>();
				solutions.add(score.getKey().gety());
			}
		}
		Collections.shuffle(solutions, new Random(System.nanoTime()));
		for(O solution: solutions)
			System.out.println(helper.Phi(example.getInputData(), solution));
		return solutions.get(0);
	}

	/**
	 * 
	 * @param data The data for which the answer needs to be predicted.
	 * @return It returns one of the output values that fits best for this input
	 */
	public O predict(I data) {
		return helper.getBestMapping(data, this.getW());
	}

	private float totalScore(HashMap<M, Float> w, O y_accent, I x, O y, boolean best) {
		if (y == null || best) {
			return score(w, y_accent, x);
		} else {
			return score(w, y_accent, x) + cost(y, y_accent);
		}

	}

	private float score(HashMap<M, Float> w, O Y_accent, I X) {
		SGD_helper<I, M, O> helper = this.getSGDHelper();
		float score = 0;
		HashMap<M, Float> phi_value = helper.Phi(X, Y_accent);
		for (Entry<M, Float> entry : phi_value.entrySet()) {
			score += w.getOrDefault(entry.getKey(), (float) 0) * entry.getValue();
		}
		return score;

	}

	private float cost(O y, O y_accent) {
		if (y.equals(y_accent)) {
			return 0;
		}
		return 1;
	}

	private HashMap<Tuple<I, O>, Float> getAllScores(HashMap<M, Float> w, I x, O y, boolean best) {
		HashMap<Tuple<I, O>, Float> result = new HashMap<>();
		for (O y_accent : getPossibleValues(x, y, best)) {
			Tuple<I, O> tuple = new Tuple<I, O>(x, y_accent);
			result.put(tuple, totalScore(w, y_accent, x, y, best));
		}
		return result;
	}

	private Entry<Tuple<I, O>, Float> getMaxScore(HashMap<M, Float> w, I x, O y) {
		HashMap<Tuple<I, O>, Float> allScores = getAllScores(w, x, y, false);
		float max = Float.NEGATIVE_INFINITY;
		List<Entry<Tuple<I, O>, Float>> biggest = new ArrayList<>();
		for (Entry<Tuple<I, O>, Float> entry : allScores.entrySet()) {
			if (Math.abs(entry.getValue() - max) < 0.0001) {
				biggest.add(entry);
			} else if (entry.getValue() > max) {
				max = entry.getValue();
				biggest = new ArrayList<>();
				biggest.add(entry);
			}
		}
		long seed = System.nanoTime();
		Collections.shuffle(biggest, new Random(seed));
		return biggest.get(0);
	}

	private Collection<O> getPossibleValues(I x, O y, boolean best) {
		SGD_helper<I, M, O> helper = this.getSGDHelper();
		if(best)
			return helper.getPossibleOutputs(x, y, this.w);
		return helper.getPossibleOutputs(x, null, this.w);
	}

	private HashMap<M, Float> getW() {
		return this.w;
	}

	private HashMap<M, Float> w = new HashMap<>();

	private SGD_helper<I, M, O> getSGDHelper() {
		return helper;
	}

	private void setSGDHelper(SGD_helper<I, M, O> helper) {
		this.helper = helper;
	}

	private SGD_helper<I, M, O> helper;
}
