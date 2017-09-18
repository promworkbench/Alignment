package nl.tue.alignment;

import java.util.Arrays;

public class SyncProductImpl implements SyncProduct {

	protected final int bm;

	protected final String[] transitions;

	protected final String[] places;

	protected final int[] cost;

	protected final byte[][] input;

	protected final byte[][] output;

	protected byte[] initMarking;

	protected byte[] finalMarking;

	protected final String label;

	public SyncProductImpl(String label, short numTrans, short numPlaces) {
		this.label = label;
		this.transitions = new String[numTrans];
		this.cost = new int[numTrans];

		this.places = new String[numTrans];

		this.bm = 1 + (numPlaces - 1) / 8;

		input = new byte[numTransitions()][bm];
		output = new byte[numTransitions()][bm];
		initMarking = new byte[2 * bm];
		finalMarking = new byte[2 * bm];

	}

	public SyncProductImpl(String label, String[] transitions, String[] places, int[] cost) {
		this.label = label;
		this.transitions = transitions;
		this.places = places;
		this.cost = cost;

		this.bm = 1 + (numPlaces() - 1) / 8;

		input = new byte[numTransitions()][bm];
		output = new byte[numTransitions()][bm];
		initMarking = new byte[2 * bm];
		finalMarking = new byte[2 * bm];

	}

	public short numTransitions() {
		return (short) transitions.length;
	}

	public short numPlaces() {
		return (short) places.length;
	}

	public void setInput(int t, int... plist) {
		input[t] = new byte[bm];
		addToInput(t, plist);
	}

	public void addToInput(int t, int... plist) {
		for (int p : plist) {
			input[t][p >>> 3] |= Utils.BYTEHIGHBIT >>> (p & 7);
		}
	}

	public void setOutput(int t, int... plist) {
		output[t] = new byte[bm];
		addToOutput(t, plist);
	}

	public void addToOutput(int t, int... plist) {
		for (int p : plist) {
			output[t][p >>> 3] |= Utils.BYTEHIGHBIT >>> (p & 7);
		}
	}

	public byte[] getInput(short transition) {
		return input[transition];
	}

	public byte[] getOutput(short transition) {
		return output[transition];
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
		this.initMarking = new byte[2 * bm];
		addToInitialMarking(places);
	}

	public void addToInitialMarking(int... places) {
		for (int p : places) {
			byte mask = (byte) (Utils.BYTEHIGHBIT >>> (p & 7));
			if ((initMarking[p >>> 3] & mask) == 0) {
				// change the low bit to 1
				initMarking[p >>> 3] |= mask;
			} else {
				if ((initMarking[bm + (p >>> 3)] & mask) == 0) {
					// change the low bit to 0
					// change the high bit to 1
					initMarking[(p >>> 3)] &= ~mask;
					initMarking[bm + (p >>> 3)] |= mask;
				} else {
					// more than 3 tokens impossible. 
					// ignore.
				}
			}
		}
	}

	public void setFinalMarking(int... places) {
		this.finalMarking = new byte[2 * bm];
		addToFinalMarking(places);
	}

	public void addToFinalMarking(int... places) {
		for (int p : places) {
			byte mask = (byte) (Utils.BYTEHIGHBIT >>> (p & 7));
			if ((finalMarking[p >>> 3] & mask) == 0) {
				// change the low bit to 1
				finalMarking[p >>> 3] |= mask;
			} else {
				if ((finalMarking[bm + (p >>> 3)] & mask) == 0) {
					// change the low bit to 0
					// change the high bit to 1
					finalMarking[(p >>> 3)] &= ~mask;
					finalMarking[bm + (p >>> 3)] |= mask;
				} else {
					// more than 3 tokens impossible. 
					// ignore.
				}
			}
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

}
