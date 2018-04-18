package nl.tue.alignment;

import java.util.Arrays;

import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

public abstract class ReplayerParameters {

	public static enum Algorithm {
		DIJKSTRA, ASTAR, ASTARWITHMARKINGSPLIT;
	}

	final Algorithm algorithm; // which algorithm
	final boolean moveSort; // moveSort on total order
	final boolean queueSort; // queue sorted "depth-first"
	final boolean preferExact; // prefer Exact solution
	final int nThreads; // do multithreading
	final boolean useInt; //  use Integer
	final Debug debug; // debug level
	final int timeoutMilliseconds; // timeout for the log
	final short[] initialSplits; // initial bins for LPbased replayer
	final int maximumNumberOfStates; // limit for the number of states
	final boolean partiallyOrderEvents;

	private ReplayerParameters(Algorithm algorithm, boolean moveSort, boolean queueSort, boolean preferExact,
			int multiThread, boolean useInt, Debug debug, int timeoutMilliseconds, int maximumNumberOfStates,
			boolean partiallyOrderEvents, short... initialSplits) {
		this.algorithm = algorithm;
		this.moveSort = moveSort;
		this.queueSort = queueSort;
		this.preferExact = preferExact;
		this.nThreads = multiThread;
		this.useInt = useInt;
		this.debug = debug;
		this.timeoutMilliseconds = timeoutMilliseconds;
		this.maximumNumberOfStates = maximumNumberOfStates;
		this.initialSplits = initialSplits;
		Arrays.sort(this.initialSplits);
		this.partiallyOrderEvents = partiallyOrderEvents;
	}

	public final static class Default extends ReplayerParameters {
		public Default() {
			super(Algorithm.ASTARWITHMARKINGSPLIT, true, true, true, 1, false, Debug.NONE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false);
		}

		public Default(int nThreads, Debug debug) {
			super(Algorithm.ASTARWITHMARKINGSPLIT, true, true, true, nThreads, false, debug, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false);
		}
	}

	public final static class AStar extends ReplayerParameters {
		public AStar() {
			super(Algorithm.ASTAR, true, true, true, Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false,
					Debug.NONE, Integer.MAX_VALUE, Integer.MAX_VALUE, false);
		}

		public AStar(Debug debug) {
			super(Algorithm.ASTAR, true, true, true, Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false,
					debug, Integer.MAX_VALUE, Integer.MAX_VALUE, false);
		}

		public AStar(boolean moveSort, boolean queueSort, boolean preferExact, int nThreads, boolean useInt,
				Debug debug, int timeoutMilliseconds, int maximumNumberOfStates, boolean partiallyOrderEvents) {
			super(Algorithm.ASTAR, moveSort, queueSort, preferExact, nThreads, useInt, debug, timeoutMilliseconds,
					maximumNumberOfStates, partiallyOrderEvents);
		}
	}

	public final static class AStarWithMarkingSplit extends ReplayerParameters {
		public AStarWithMarkingSplit() {
			super(Algorithm.ASTARWITHMARKINGSPLIT, false, true, true,
					Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false, Debug.NONE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false);
		}

		public AStarWithMarkingSplit(Debug debug) {
			super(Algorithm.ASTARWITHMARKINGSPLIT, false, true, true,
					Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false, debug, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false);
		}

		public AStarWithMarkingSplit(boolean moveSort, int nThreads, boolean useInt, Debug debug,
				int timeoutMilliseconds, int maximumNumberOfStates, boolean partiallyOrderEvents,
				short... initialSplits) {
			super(Algorithm.ASTARWITHMARKINGSPLIT, moveSort, true, true, nThreads, useInt, debug, timeoutMilliseconds,
					maximumNumberOfStates, partiallyOrderEvents, initialSplits);
		}
	}

	public final static class Dijkstra extends ReplayerParameters {
		public Dijkstra() {
			super(Algorithm.DIJKSTRA, false, true, true, 1, false, Debug.NONE, Integer.MAX_VALUE, Integer.MAX_VALUE,
					false);
		}

		public Dijkstra(Debug debug) {
			super(Algorithm.DIJKSTRA, false, true, true, 1, false, debug, Integer.MAX_VALUE, Integer.MAX_VALUE, false);
		}

		public Dijkstra(boolean moveSort, boolean queueSort, int nThreads, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, boolean partiallyOrderEvents) {
			super(Algorithm.DIJKSTRA, moveSort, queueSort, true, nThreads, false, debug, timeoutMilliseconds,
					maximumNumberOfStates, partiallyOrderEvents);
		}
	}

}