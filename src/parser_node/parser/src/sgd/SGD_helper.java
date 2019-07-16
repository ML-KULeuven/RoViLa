package sgd;
import java.util.Collection;
import java.util.HashMap;

import main.LogicalExpression;

public interface SGD_helper<I, M, O> {
	/**
	 * 
	 * @param 	x
	 * 			The input
	 * @param 	y
	 * 			The (possible) output for this value
	 * @return	The score of this input/output combination
	 */
	public HashMap<M, Float> Phi(I x, O y);
	
	/**
	 * 
	 * @param 	x
	 * 			Input value
	 * @return 	A list of all possible solutions
	 */
	public Collection<O> getPossibleOutputs(I x, O y, HashMap<M, Float> w);
	
	public O getBestMapping(I x, HashMap<M, Float> w);
}
