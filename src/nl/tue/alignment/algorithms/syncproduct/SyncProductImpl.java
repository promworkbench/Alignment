package nl.tue.alignment.algorithms.syncproduct;

import java.util.Arrays;

public class SyncProductImpl implements SyncProduct {

	private static final short[] EMPTY = new short[0];

	protected final String[] transitions;

	protected final short[] eventNumbers;

	protected final short[] ranks;

	protected final String[] places;

	protected final int[] cost;

	protected final short[][] input;

	protected final short[][] output;

	protected byte[] initMarking;

	protected byte[] finalMarking;

	protected final String label;

	private final byte[] types;

	private final short numEvents;

	private final int[] moves;

	private final short numClasses;

	private final short numModelMoves;

	public SyncProductImpl(String label, short numClasses, String[] transitions, String[] places, short[] eventNumbers,
			byte[] types, int[] moves, int[] cost) {
		this(label, numClasses, transitions, places, eventNumbers, eventNumbers, types, moves, cost);
	}

	public SyncProductImpl(String label, short numClasses, String[] transitions, String[] places, short[] eventNumbers,
			short[] ranks, byte[] types, int[] moves, int[] cost) {
		this.eventNumbers = eventNumbers;
		this.label = label;
		this.transitions = transitions;
		this.places = places;
		this.types = types;
		this.moves = moves;
		this.cost = cost;
		this.ranks = ranks;

		short mx = 0;
		for (int e = 0; e < eventNumbers.length; e++) {
			if (eventNumbers[e] > mx) {
				mx = eventNumbers[e];
			}
		}
		this.numEvents = (short) (mx + 1);

		mx = 0;
		for (int e = 0; e < moves.length; e++) {
			if (types[e] == MODEL_MOVE || types[e] == TAU_MOVE) {
				mx++;
			}
		}
		this.numClasses = numClasses;
		this.numModelMoves = mx;

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

	private void setSortedArray(short[] array, short[] plist) {
		Arrays.sort(plist);
		for (int i = plist.length; i-- > 0;) {
			array[i] = plist[i];
		}
	}

	private void setSortedArray(short[] array, int[] plist) {
		Arrays.sort(plist);
		for (int i = plist.length; i-- > 0;) {
			array[i] = (short) plist[i];
		}
	}

	public void setInput(short t, short... plist) {
		input[t] = new short[plist.length];
		setSortedArray(input[t], plist);
	}

	public void setOutput(short t, short... plist) {
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

	public void addToOutput(short t, short... p) {
		output[t] = Arrays.copyOf(getOutput(t), getOutput(t).length + p.length);
		System.arraycopy(p, 0, output[t], output[t].length - p.length, p.length);
		Arrays.sort(output[t]);
	}

	public void addToInput(short t, short... p) {
		input[t] = Arrays.copyOf(getInput(t), getInput(t).length + p.length);
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
	 * For examples: place 18 marked with a single token return (marking[18 / 8] &
	 * (Utils.FLAG >>> (18 % 8))) != 0 && (marking[bm + 18 / 8] & (Utils.FLAG >>>
	 * (18 % 8))) == 0;
	 */
	public boolean isFinalMarking(byte[] marking) {
		// for full alignments:
		return Arrays.equals(marking, finalMarking);

	}

	public String getLabel() {
		return label;
	}

	public short getEventOf(short transition) {
		return eventNumbers[transition];
	}

	public void setEventOf(short transition, short event) {
		eventNumbers[transition] = event;
	}

	public byte getTypeOf(short transition) {
		return types[transition];
	}

	public short numEvents() {
		return numEvents;
	}

	public short getRankOf(short transition) {
		return ranks[transition];
	}

	public void setRankOf(short transition, short rank) {
		ranks[transition] = rank;
	}

	public int getMoveOf(short transition) {
		return moves[transition];
	}

	public short numEventClasses() {
		return numClasses;
	}

	public int numModelMoves() {
		return numModelMoves;
	}

}
