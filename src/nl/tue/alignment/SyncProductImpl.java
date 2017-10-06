package nl.tue.alignment;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Arrays;

public class SyncProductImpl implements SyncProduct {

	private static final short[] EMPTY = new short[0];

	protected final String[] transitions;

	protected final short[] eventNumbers;

	protected final String[] places;

	protected final int[] cost;

	protected final short[][] input;

	protected final short[][] output;

	protected byte[] initMarking;

	protected byte[] finalMarking;

	protected final String label;

	public SyncProductImpl(String label, short numTrans, short numPlaces) {
		if (numTrans > MAXTRANS) {
			throw new RuntimeException("More than " + MAXTRANS + " transitions in a synchronous product is not allowed");
		}
		this.label = label;
		this.transitions = new String[numTrans];
		this.cost = new int[numTrans];

		this.places = new String[numTrans];

		input = new short[numTransitions()][];
		output = new short[numTransitions()][];
		eventNumbers = new short[numTrans];
		Arrays.fill(eventNumbers, (short) -1);

		initMarking = new byte[numPlaces()];
		finalMarking = new byte[numPlaces()];

	}

	public SyncProductImpl(String label, String[] transitions, String[] places, short[] eventNumbers, int[] cost) {
		this.eventNumbers = eventNumbers;
		this.label = label;
		this.transitions = transitions;
		this.places = places;
		this.cost = cost;

		input = new short[numTransitions()][];
		output = new short[numTransitions()][];

		initMarking = new byte[numPlaces()];
		finalMarking = new byte[numPlaces()];

	}

	public short numTransitions() {
		return (short) transitions.length;
	}

	public short numPlaces() {
		return (short) places.length;
	}

	private void setSortedArray(short[] array, int[] plist) {
		Arrays.sort(plist);
		for (int i = plist.length; i-- > 0;) {
			array[i] = (short) plist[i];
		}
	}

	public void setInput(short t, int... plist) {
		input[t] = new short[plist.length];
		setSortedArray(input[t], plist);
	}

	public void setOutput(short t, int... plist) {
		output[t] = new short[plist.length];
		setSortedArray(output[t], plist);
	}

	public void setInput(int t, int... plist) {
		input[t] = new short[plist.length];
		setSortedArray(input[t], plist);
	}

	public void setOutput(int t, int... plist) {
		output[t] = new short[plist.length];
		setSortedArray(output[t], plist);
	}

	public void addToOutput(int t, short... p) {
		output[t] = Arrays.copyOf(getOutput((short) t), getOutput((short) t).length + p.length);
		System.arraycopy(p, 0, output[t], output[t].length - p.length, p.length);
		Arrays.sort(output[t]);
	}

	public void addToInput(int t, short... p) {
		input[t] = Arrays.copyOf(getInput((short) t), getInput((short) t).length + p.length);
		System.arraycopy(p, 0, input[t], input[t].length - p.length, p.length);
		Arrays.sort(input[t]);
	}

	public short[] getInput(short transition) {
		return input[transition] == null ? EMPTY : input[transition];
	}

	public short[] getOutput(short transition) {
		return output[transition] == null ? EMPTY : output[transition];
	}

	public byte[] getInitialMarking() {
		return initMarking;
	}

	public byte[] getFinalMarking() {
		return finalMarking;
	}

	public void setInitialMarking(byte[] marking) {
		this.initMarking = marking;
	}

	public void setFinalMarking(byte[] marking) {
		this.finalMarking = marking;
	}

	public void setInitialMarking(int... places) {
		this.initMarking = new byte[numPlaces()];
		addToInitialMarking(places);
	}

	public void addToInitialMarking(int... places) {
		for (int p : places) {
			initMarking[p]++;
		}
	}

	public void setFinalMarking(int... places) {
		this.finalMarking = new byte[numPlaces()];
		addToFinalMarking(places);
	}

	public void addToFinalMarking(int... places) {
		for (int p : places) {
			finalMarking[p]++;
		}
	}

	public int getCost(short t) {
		return cost[t];
	}

	public String getTransitionLabel(short t) {
		return transitions[t];
	}

	public String getPlaceLabel(short p) {
		return places[p];
	}

	public void setTransitionLabel(short t, String label) {
		transitions[t] = label;
	}

	public void setPlaceLabel(short p, String label) {
		places[p] = label;
	}

	/**
	 * for full alignments: return Arrays.equals(marking, finalMarking);
	 * 
	 * for prefix alignments: check only if a specific place is marked.
	 * 
	 * For examples: place 18 marked with a single token return (marking[18 / 8]
	 * & (Utils.FLAG >>> (18 % 8))) != 0 && (marking[bm + 18 / 8] & (Utils.FLAG
	 * >>> (18 % 8))) == 0;
	 */
	public boolean isFinalMarking(byte[] marking) {
		// for full alignments:
		return Arrays.equals(marking, finalMarking);

	}

	public String getLabel() {
		return label;
	}

	public void toTpn(OutputStreamWriter stream) throws IOException {
		for (short p = 0; p < numPlaces(); p++) {
			stream.write("place \"place_" + p);
			stream.write("\"");
			if (getInitialMarking()[p] > 0) {
				stream.write("init " + getInitialMarking()[p]);
			}
			stream.write(";\n");
		}
		for (short t = 0; t < numTransitions(); t++) {
			stream.write("trans \"t_" + t);
			stream.write("\"~\"");
			stream.write(getTransitionLabel(t));
			stream.write("\" in ");
			for (short p : getInput(t)) {
				stream.write(" \"place_" + p);
				stream.write("\"");
			}
			stream.write(" out ");
			for (short p : getOutput(t)) {
				stream.write(" \"place_" + p);
				stream.write("\"");
			}
			stream.write(";\n");
		}

	}

	public short getEventOf(short transition) {
		return eventNumbers[transition];
	}

	public void setEventOf(short transition, short event) {
		eventNumbers[transition] = event;
	}

}
