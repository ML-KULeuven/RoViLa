package sgd_implementation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import main.LogicalExpression;
import main.LogicalExpressionEmpty;
import main.LogicalExpressionGeneral;
import sgd.SGD_helper;
import test.ContextValidation;

public class SGD_Helper_Implementation implements SGD_helper<String, String, LogicalExpression> {

	private final int BEAM_SIZE = 1000;

	private final LogicalExpressionGeneral[] templates = { new LogicalExpressionGeneral("Put", 1),
			new LogicalExpressionGeneral("On", 2), new LogicalExpressionGeneral("Between", 3),
			new LogicalExpressionGeneral("FrontOf", 2), new LogicalExpressionGeneral("Behind", 2),
			new LogicalExpressionGeneral("LeftOf", 2), new LogicalExpressionGeneral("RightOf", 2),
			new LogicalExpressionGeneral("Red", 1), new LogicalExpressionGeneral("Brown", 1),
			new LogicalExpressionGeneral("Blue", 1), new LogicalExpressionGeneral("Yellow", 1),
			new LogicalExpressionGeneral("Gray", 1), new LogicalExpressionGeneral("Purple", 1),
			new LogicalExpressionGeneral("Cyan", 1), new LogicalExpressionGeneral("Green", 1),
			new LogicalExpressionGeneral("Block", 1), new LogicalExpressionGeneral("Cylinder", 1),
			new LogicalExpressionGeneral("Sphere", 1), new LogicalExpressionGeneral("Metal", 1),
			new LogicalExpressionGeneral("Rubber", 1) };

	@Override
	public HashMap<String, Float> Phi(String x, LogicalExpression y) {
		return y.getFeatures();
	}

	@Override
	public Collection<LogicalExpression> getPossibleOutputs(String x, LogicalExpression y, HashMap<String, Float> w) {
		System.out.println(x);
		Collection<LogicalExpression> result = new ArrayList<>();
		int beam = BEAM_SIZE / 2;
		int count = 0;
		while (result.size() == 0) {
			beam = beam * 2;
			Collection<ArrayList<LogicalExpression>> mappingWords = mapping_words(x, y, w, beam);
			result = merge(mappingWords, y, w, beam);
			count++;
			if (count > 1) {
				if (y != null)
					System.out.println(y.toString());
				System.out.println("BEAM INCREASED " + Integer.toString(beam));
			}
		}
		return result;
	}

	public String parse(String sentence, HashMap<String, Float> w) {
		Collection<LogicalExpression> result = new ArrayList<>();
		int beam = BEAM_SIZE / 2;
		int count = 0;
		while (result.size() == 0) {
			beam = beam * 2;
			Collection<ArrayList<LogicalExpression>> mappingWords = mapping_words(sentence, null, w, beam);
			float max_score = 0;
			Collection<ArrayList<LogicalExpression>> mappings = new ArrayList<>();
			for (ArrayList<LogicalExpression> mapping : mappingWords) {
				float cost = getCostMappings(mapping, w);
				if (max_score < cost) {
					max_score = cost;
					mappings = new ArrayList<>();
				}
				if (max_score == cost) {
					mappings.add(mapping);
				}
			}
			// System.out.println(mappings.size());
			result = merge(mappings, null, w, beam);
			count++;
			if (count > 1) {
				System.out.println("BEAM INCREASED " + Integer.toString(beam));
			}
		}
		// System.out.println(result);
		// System.out.println(result.size());
		Optional<LogicalExpression> best = result.stream()
				.sorted((LogicalExpression l1, LogicalExpression l2) -> Float.compare(getCost(l2, w), getCost(l1, w)))
				.findFirst();
		return best.get().toString();
	}

	public String parseContext(String sceneId, String sentence, HashMap<String, Float> w) {
		int beam = BEAM_SIZE / 2;
		int count = 0;
		Optional<LogicalExpression> solution = Optional.empty();
		while (!solution.isPresent()) {
			beam = beam * 2;
			Collection<ArrayList<LogicalExpression>> mappingWords = mapping_words(sentence, null, w, beam);
			float max_score = 0;
			ArrayList<ArrayList<ArrayList<LogicalExpression>>> mapping_groups = new ArrayList<>();
			ArrayList<ArrayList<LogicalExpression>> mappings = new ArrayList<>();
			for(ArrayList<LogicalExpression> mapping : mappingWords) {
				float cost = getCostMappings(mapping, w);
				if(max_score < cost) {
					max_score = cost;
					mapping_groups.add(0, mappings);
					mappings = new ArrayList<>();
				}
				if(max_score == cost) {
					mappings.add(mapping);
				}
			}
			
			mapping_groups.add(0, mappings);
			for(int i = 0; i < mapping_groups.size() && !solution.isPresent(); i++) {
				Collection<LogicalExpression> result = merge(mapping_groups.get(i), null, w, beam);
				solution = result.stream().sorted((LogicalExpression l1, LogicalExpression l2) -> Float.compare(getCost(l2, w), getCost(l1, w))).filter(x -> ContextValidation.validate(sceneId, x.callFormat())).findFirst();
			}			
			count++;
			if (count > 1) {
				System.out.println("BEAM INCREASED " + Integer.toString(beam));
			}
		}
		//System.out.println(solution.get().getFeatures());
		return solution.get().toString();
	}

	public Collection<ArrayList<LogicalExpression>> mapping_words(String x, LogicalExpression y,
			HashMap<String, Float> w, int beam_size) {
		String words[] = tokenize(x);
		Comparator<ArrayList<LogicalExpression>> byCost = (ArrayList<LogicalExpression> l1,
				ArrayList<LogicalExpression> l2) -> Float.compare(getCostMappings(l2, w), getCostMappings(l1, w));
		PriorityQueue<ArrayList<LogicalExpression>> queue1 = new PriorityQueue<>(byCost);
		PriorityQueue<ArrayList<LogicalExpression>> queue2 = new PriorityQueue<>(byCost);
		queue1.add(new ArrayList<>());
		for (String word : words) {
			for (int i = 0; (i < beam_size || (y != null && i < 16000)) && !queue1.isEmpty(); i++) {
				ArrayList<LogicalExpression> expressions = queue1.poll();
				for (LogicalExpressionGeneral expr : templates) {
					expr = expr.deepCopy();
					expr.mapOn(word);
					ArrayList<LogicalExpression> newList = copy(expressions);
					newList.add(expr);
					if (validate(y, newList))
						queue2.add(newList);
				}
				ArrayList<LogicalExpression> newList = copy(expressions);
				newList.add(new LogicalExpressionEmpty(word));
				queue2.add(newList);
			}
			queue1 = queue2;
			queue2 = new PriorityQueue<>(byCost);
		}
		return queue1;
	}

	public Collection<LogicalExpression> merge(Collection<ArrayList<LogicalExpression>> mappings,
			LogicalExpression solution, HashMap<String, Float> w, int beam_size) {
		Comparator<List<LogicalExpressionGeneral>> byCost = (List<LogicalExpressionGeneral> l1,
				List<LogicalExpressionGeneral> l2) -> Float.compare(getCostGeneral(l2, w), getCostGeneral(l1, w));
		PriorityQueue<List<LogicalExpressionGeneral>> queue1 = new PriorityQueue<>(byCost);
		PriorityQueue<List<LogicalExpressionGeneral>> queue2 = new PriorityQueue<>(byCost);
		HashSet<List<LogicalExpressionGeneral>> set = new HashSet<>();
		int amountComponents = solution == null ? 0 : componentSize(solution);
		queue1.addAll(mappings.stream().map(x -> clean(x)).filter(x -> x.size() > 0)
				.filter(x -> amountComponents == 0 || x.size() == amountComponents).collect(Collectors.toList()));
		HashSet<LogicalExpression> final_values = new HashSet<>();
		while (queue1.size() > 0) {
			for (int i = 0; (i < beam_size || (solution != null && i < 16000)) && !queue1.isEmpty(); i++) {
				List<LogicalExpressionGeneral> values = queue1.poll();
				ArrayList<List<LogicalExpressionGeneral>> result = oneMergeStep(values, solution);
				for (List<LogicalExpressionGeneral> v : result) {
					if (v.size() == 1) {
						if (solution != null && !solution.toString().equals(v.get(0).toString()))
							continue;
						final_values.add(v.get(0));
						set.add(v);
						continue;
					}
					if (set.contains(v))
						continue;
					queue2.add(v);
					set.add(v);
				}
			}
			queue1 = queue2;
			queue2 = new PriorityQueue<>(byCost);
		}
		return final_values;

	}

	private Collection<LogicalExpression> merge(Collection<ArrayList<LogicalExpression>> mappings,
			LogicalExpression solution, HashMap<String, Float> w) {
		int amountComponents = solution == null ? 0 : componentSize(solution);
		List<List<LogicalExpressionGeneral>> cleaned_mappings = mappings.stream().map(x -> clean(x))
				.filter(x -> x.size() > 0).filter(x -> amountComponents == 0 || x.size() == amountComponents)
				.collect(Collectors.toList());
		HashSet<LogicalExpression> final_values = new HashSet<>();
		for (List<LogicalExpressionGeneral> mapping : cleaned_mappings) {
			final_values.addAll(merge(mapping, solution, w));
		}
		return final_values;
	}

	private Collection<LogicalExpressionGeneral> merge(List<LogicalExpressionGeneral> mappings,
			LogicalExpression solution, HashMap<String, Float> w) {
		HashSet<LogicalExpressionGeneral> result = new HashSet<>();
		HashSet<List<LogicalExpressionGeneral>> values = new HashSet<>();
		values.add(mappings);
		while (values.size() > 0) {
			List<LogicalExpressionGeneral> value = values.iterator().next();
			values.remove(value);
			if (value.size() == 1) {
				if (solution != null && !solution.toString().equals(value.get(0).toString()))
					continue;
				result.add(value.get(0));
				continue;
			}
			values.addAll(oneMergeStep(value, solution));
		}
		return result;
	}

	private List<LogicalExpressionGeneral> clean(Collection<LogicalExpression> expression) {
		List<LogicalExpressionGeneral> cleaned = expression.stream().filter(x -> x instanceof LogicalExpressionGeneral)
				.map(x -> (LogicalExpressionGeneral) x).collect(Collectors.toList());
		if (cleaned.size() == 0)
			return cleaned;
		((LogicalExpressionGeneral) cleaned.get(0))
				.addEmptyMapping(expression.stream().filter(x -> x instanceof LogicalExpressionEmpty)
						.map(x -> (LogicalExpressionEmpty) x).collect(Collectors.toList()));
		return cleaned;
	}

	public ArrayList<List<LogicalExpressionGeneral>> oneMergeStep(List<LogicalExpressionGeneral> mappings,
			LogicalExpression solution) {
		ArrayList<List<LogicalExpressionGeneral>> expressions = new ArrayList<>();
		int max = mappings.size();
		for (int i = 0; i < max - 1; i++) {
			int size = mappings.get(i + 1).size();
			Collection<LogicalExpressionGeneral> mergeResult = new ArrayList<>();
			switch (size) {
			case 1:
				// mergeResult = merge(mappings.get(i), mappings.get(i+1));
				break;
			case 2:
				if (i + 2 < max)
					mergeResult = merge(mappings.get(i), mappings.get(i + 1), mappings.get(i + 2));
				break;
			case 3:
				if (i + 3 < max)
					mergeResult = merge(mappings.get(i), mappings.get(i + 1), mappings.get(i + 2), mappings.get(i + 3));
				break;
			default:
				throw new IllegalStateException();
			}
			for (LogicalExpressionGeneral merge : mergeResult) {
				if (!validate(solution, merge))
					continue;
				List<LogicalExpressionGeneral> new_list = copyG(mappings.subList(0, i));
				new_list.add(merge);
				new_list.addAll(copyG(mappings.subList(i + size + 1, mappings.size())));
				new_list = copyG(new_list);
				expressions.add(new_list);
			}
			mergeResult = new ArrayList<>();
			if (mappings.get(i).size() == 1) {
				mergeResult = merge(mappings.get(i), mappings.get(i + 1));
			}
			for (LogicalExpressionGeneral merge : mergeResult) {
				if (!validate(solution, merge))
					continue;
				List<LogicalExpressionGeneral> new_list = copyG(mappings.subList(0, i));
				new_list.add(merge);
				new_list.addAll(copyG(mappings.subList(i + 2, mappings.size())));
				new_list = copyG(new_list);
				expressions.add(new_list);
			}
		}
		return expressions;
	}

	public Collection<LogicalExpressionGeneral> merge(LogicalExpressionGeneral expr1, LogicalExpressionGeneral expr2) {
		ArrayList<LogicalExpressionGeneral> result = new ArrayList<>();
		if (expr1.free() && expr1.size() == 1) {
			LogicalExpressionGeneral newExpr = expr1.deepCopy();
			newExpr.addExpressions(new LogicalExpressionGeneral[] { expr2.deepCopy() });
			result.add(newExpr);
		}
		/**
		 * if (expr2.free()) { LogicalExpressionGeneral newExpr = expr2.deepCopy();
		 * newExpr.addExpressions(new LogicalExpressionGeneral[] { expr1.deepCopy() });
		 * result.add(newExpr); }
		 **/
		return result;
	}

	public ArrayList<LogicalExpressionGeneral> merge(LogicalExpressionGeneral expr1, LogicalExpressionGeneral expr2,
			LogicalExpressionGeneral expr3) {
		ArrayList<LogicalExpressionGeneral> result = new ArrayList<>();
		if (expr2.free()) {
			LogicalExpressionGeneral newExpr = expr2.deepCopy();
			newExpr.addExpressions(new LogicalExpressionGeneral[] { expr1.deepCopy(), expr3.deepCopy() });
			result.add(newExpr);
		}
		return result;

	}

	private int componentSize(LogicalExpression expr) {
		int count = 0;
		for (int val : expr.components().values())
			count += val;
		return count;
	}

	public ArrayList<LogicalExpressionGeneral> merge(LogicalExpressionGeneral expr1, LogicalExpressionGeneral expr2,
			LogicalExpressionGeneral expr3, LogicalExpressionGeneral expr4) {
		ArrayList<LogicalExpressionGeneral> result = new ArrayList<>();
		if (expr2.free()) {
			LogicalExpressionGeneral newExpr = expr2.deepCopy();
			newExpr.addExpressions(
					new LogicalExpressionGeneral[] { expr1.deepCopy(), expr3.deepCopy(), expr4.deepCopy() });
			result.add(newExpr);
		}
		return result;

	}

	public ArrayList<LogicalExpression> copy(ArrayList<LogicalExpression> new_list) {
		ArrayList<LogicalExpression> values = new ArrayList<>();
		for (LogicalExpression expr : new_list) {
			values.add(expr.deepCopy());
		}
		return values;
	}

	public ArrayList<LogicalExpressionGeneral> copyG(List<LogicalExpressionGeneral> new_list) {
		ArrayList<LogicalExpressionGeneral> values = new ArrayList<>();
		for (LogicalExpressionGeneral expr : new_list) {
			values.add(expr.deepCopy());
		}
		return values;
	}

	public float getCostMappings(List<LogicalExpression> l1, HashMap<String, Float> w) {
		float cost = 0;
		for (LogicalExpression expr : l1) {
			cost += getCostMappings(expr, w);
		}
		return cost;
	}

	public float getCostGeneral(List<LogicalExpressionGeneral> l1, HashMap<String, Float> w) {
		float cost = 0;
		for (LogicalExpression expr : l1) {
			if (expr.free())
				cost += getCostMappings(expr, w);
			else
				cost += getCost(expr, w);
		}
		return cost;
	}

	public float getCostMappings(LogicalExpression expr, HashMap<String, Float> w) {
		float cost = 0;
		HashMap<String, Float> features = expr.getFeatures();
		for (Entry<String, Float> entry : features.entrySet()) {
			if (!entry.getKey().contains(" -> "))
				continue;
			cost += w.getOrDefault(entry.getKey(), (float) 0) * entry.getValue();
		}
		return cost;
	}

	public float getCost(LogicalExpression expr, HashMap<String, Float> w) {
		float cost = 0;
		HashMap<String, Float> features = expr.getFeatures();
		for (Entry<String, Float> entry : features.entrySet()) {
			cost += w.getOrDefault(entry.getKey(), (float) 0) * entry.getValue();
		}
		return cost;
	}

	public boolean validate(LogicalExpression solution, Collection<LogicalExpression> expressions) {
		if (solution == null)
			return true;
		HashMap<String, Integer> components = solution.components();
		int count = 0;
		for (LogicalExpression expr : expressions) {
			String name = expr.getName();
			if (name == null)
				continue;
			count++;
			int amount = components.getOrDefault(name, 0);
			if (amount == 0)
				return false;
			components.put(name, amount - 1);
		}
		return count > 0;
	}

	public boolean validate(LogicalExpression solution, LogicalExpression expression) {
		if (solution == null)
			return true;
		return solution.toString().contains(expression.toString());
	}

	private String[] tokenize(String input) {
		ArrayList<String> result = new ArrayList<>();
		input = input.toLowerCase();
		for (String token : input.split(" ")) {
			token = token.trim();
			if (token.length() == 0 || token.equals("the"))
				continue;
			result.add(token);
		}
		return result.toArray(new String[result.size()]);
	}

	@Override
	public LogicalExpression getBestMapping(String x, HashMap<String, Float> w) {
		// TODO Auto-generated method stub
		return null;
	}

}
