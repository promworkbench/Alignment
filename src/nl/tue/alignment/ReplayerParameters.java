package nl.tue.alignment;

import nl.tue.alignment.Replayer.Algorithm;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

public class ReplayerParameters {
	final Algorithm algorithm; // which algorithm
	final boolean moveSort; // moveSort on total order
	final boolean queueSort; // queue sorted "depth-first"
	final boolean preferExact; // prefer Exact solution
	final boolean multiThread; // do multithreading
	final boolean useInt; //  use Integer
	final Debug debug;

	public ReplayerParameters(Algorithm algorithm, boolean moveSort, boolean queueSort, boolean preferExact,
			boolean multiThread, boolean useInt, Debug debug) {
		this.algorithm = algorithm;
		this.moveSort = moveSort;
		this.queueSort = queueSort;
		this.preferExact = preferExact;
		this.multiThread = multiThread;
		this.useInt = useInt;
		this.debug = debug;
	}

	public ReplayerParameters() {
		this(Algorithm.ASTARWITHMARKINGSPLIT, true, true, true, false, false, Debug.NONE);
	}
}