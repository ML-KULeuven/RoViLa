package sgd;


public class TrainingExample<I, O> {
	/**
	 * 
	 * @param 	inputData
	 * 			The input for which you want to train.
	 * @param 	correctOutput
	 * 			The correct output for the given input.
	 */
	public TrainingExample(I inputData, O correctOutput){
		setInputData(inputData);
		setCorrectOutput(correctOutput);
	}
	/**
	 * 
	 * @return the input for this trainingsexample
	 */
	public I getInputData() {
		return inputData;
	}
	private void setInputData(I inputData) {
		this.inputData = inputData;
	}
	private I inputData;
	
	/**
	 * 
	 * @return the correct output for this trainingsexample
	 */
	public O getCorrectOutput() {
		return correctOutput;
	}
	private void setCorrectOutput(O correctOutput) {
		this.correctOutput = correctOutput;
	}
	
	private O correctOutput;

}
