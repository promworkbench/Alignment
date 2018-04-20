package nl.tue.alignment.algorithms.constraints;

import java.util.Arrays;

public class Constraint {

	private final int[] inputLabelOccurrence;

	private final int[] outputLabelOccurrence;

	private int value = 0;

	private int threshold;

	private String[] colNames = null;

	Constraint(int classes, int threshold, String[] colNames) {
		this(classes, threshold);
		this.colNames = colNames;
	}

	public Constraint(int classes, int threshold) {
		this.threshold = threshold;
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
		value = threshold;
	}

	public boolean satisfiedAfterOccurence(short label) {
		value += inputLabelOccurrence[label];
		value -= outputLabelOccurrence[label];
		return value >= threshold;
	}

	public void add(Constraint constraint) {
		for (short l = 0; l < inputLabelOccurrence.length; l++) {
			inputLabelOccurrence[l] += constraint.inputLabelOccurrence[l];
			outputLabelOccurrence[l] += constraint.outputLabelOccurrence[l];
		}
	}

	public int hashCode() {
		return threshold + 31 * Arrays.hashCode(inputLabelOccurrence)
				+ 31 * 31 * Arrays.hashCode(outputLabelOccurrence);
	}

	public boolean equals(Object o) {
		if (o instanceof Constraint) {
			Constraint c = (Constraint) o;
			return threshold == c.threshold && Arrays.equals(inputLabelOccurrence, c.inputLabelOccurrence)
					&& Arrays.equals(outputLabelOccurrence, c.outputLabelOccurrence);
		}
		return false;
	}

	public int getInputValue(short label) {
		return inputLabelOccurrence[label];
	}

	public int getOutputValue(short label) {
		return outputLabelOccurrence[label];
	}

	public String toString() {
		StringBuilder b = new StringBuilder();
		boolean first = true;
		for (int i = 0; i < inputLabelOccurrence.length; i++) {
			if (inputLabelOccurrence[i] > 0) {
				b.append((!first ? " + " : "") + (inputLabelOccurrence[i]>1? inputLabelOccurrence[i]+" ":"") + (colNames == null ? i : colNames[i]));
				first = false;
			}
		}
		if (b.length() == 0) {
			b.append("0");
		}
		b.append(" >= ");
		first = true;
		for (int i = 0; i < outputLabelOccurrence.length; i++) {
			if (outputLabelOccurrence[i] > 0) {
				b.append((!first ? " + " : "") + (outputLabelOccurrence[i]>1? outputLabelOccurrence[i]+" ":"")  + (colNames == null ? i : colNames[i]));
				first = false;
			}
		}
		if (threshold != 0) {
			b.append(" " + threshold);
		}
		return b.toString();
	}
}
