package nl.tue.alignment.algorithms.datastructures;

public class Constraint {

	private final int[] inputLabelOccurrence;

	private final int[] outputLabelOccurrence;

	private int value = 0;

	public boolean isAlwaysSatisfied = false;

	public Constraint(int classes) {
		inputLabelOccurrence = new int[classes];
		outputLabelOccurrence = new int[classes];

	}

	public void addInput(short label, int value) {
		inputLabelOccurrence[label] += value;
	}

	public void addOutput(short label, int value) {
		outputLabelOccurrence[label] += value;
	}

	public void setInput(short label, int value) {
		inputLabelOccurrence[label] = value;
	}

	public void setOutput(short label, int value) {
		outputLabelOccurrence[label] = value;
	}

	public void reset() {
		value = 0;
	}

	public void setAlwaysSatisfied() {
		isAlwaysSatisfied = true;
	}

	public boolean satisfiedAfterOccurence(short label) {
		value += inputLabelOccurrence[label];
		value -= outputLabelOccurrence[label];
		return isAlwaysSatisfied || value >= 0;
	}

	public void add(Constraint constraint) {
		for (short l = 0; l < inputLabelOccurrence.length; l++) {
			inputLabelOccurrence[l] += constraint.inputLabelOccurrence[l];
			outputLabelOccurrence[l] += constraint.outputLabelOccurrence[l];
		}
		isAlwaysSatisfied |= constraint.isAlwaysSatisfied;
	}

	public int hashCode() {
		return super.hashCode();
	}

	public boolean equals(Object o) {
		return super.equals(o);
	}

	public int getInputValue(short label) {
		return inputLabelOccurrence[label];
	}

	public int getOutputValue(short label) {
		return outputLabelOccurrence[label];
	}

}
