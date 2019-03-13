package nl.tue.alignment;

import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

public abstract class ReplayerParameters {

	public static enum Algorithm {
		DIJKSTRA, ASTAR, INCREMENTALASTAR, FULL;
	}

	public final Algorithm algorithm; // which algorithm
	public final boolean moveSort; // moveSort on total order
	public final boolean queueSort; // queue sorted "depth-first"
	public final boolean preferExact; // prefer Exact solution
	public final int nThreads; // do multithreading
	public final boolean useInt; //  use Integer
	public final Debug debug; // debug level
	public final int timeoutMilliseconds; // timeout for the log
	public final int maximumNumberOfStates; // limit for the number of states
	public final int costUpperBound; // limit for the number of states
	public final boolean partiallyOrderEvents;
	public final boolean preProcessUsingPlaceBasedConstraints;
	public final int initialSplits;
	public final int maxReducedSequenceLength;

	private ReplayerParameters(Algorithm algorithm, boolean moveSort, boolean queueSort, boolean preferExact,
			int nThreads, boolean useInt, Debug debug, int timeoutMilliseconds, int maximumNumberOfStates,
			int costUpperBound, boolean partiallyOrderEvents, boolean preProcessUsingPlaceBasedConstraints,
			int initialSplits, int maxReducedSequenceLength) {
		this.algorithm = algorithm;
		this.moveSort = moveSort;
		this.queueSort = queueSort;
		this.preferExact = preferExact;
		this.nThreads = nThreads;
		this.useInt = useInt;
		this.debug = debug;
		this.timeoutMilliseconds = timeoutMilliseconds;
		this.maximumNumberOfStates = maximumNumberOfStates;
		this.costUpperBound = costUpperBound;
		this.partiallyOrderEvents = partiallyOrderEvents;
		this.preProcessUsingPlaceBasedConstraints = preProcessUsingPlaceBasedConstraints;
		this.initialSplits = initialSplits;
		this.maxReducedSequenceLength = maxReducedSequenceLength;
	}

	public final static class Default extends ReplayerParameters {
		public Default() {
			super(Algorithm.INCREMENTALASTAR, false, true, true,
					Math.max(1, Runtime.getRuntime().availableProcessors() / 4), false, Debug.NONE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, Integer.MAX_VALUE, false, true, 0, 2);
		}

		public Default(int nThreads, Debug debug) {
			super(Algorithm.INCREMENTALASTAR, false, true, true, nThreads, false, debug, Integer.MAX_VALUE,
					Integer.MAX_VALUE, Integer.MAX_VALUE, false, false, 0, 2);
		}

		public Default(int nThreads, int costUpperBound, Debug debug) {
			super(Algorithm.INCREMENTALASTAR, false, true, true, nThreads, false, debug, Integer.MAX_VALUE,
					Integer.MAX_VALUE, costUpperBound, false, false, 0, 2);
		}

	}

	public final static class AStar extends ReplayerParameters {
		public AStar() {
			super(Algorithm.ASTAR, true, true, true, Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false,
					Debug.NONE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, false, false, 0, 1);
		}

		public AStar(Debug debug) {
			super(Algorithm.ASTAR, true, true, true, Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false,
					debug, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, false, false, 0, 1);
		}

		public AStar(boolean moveSort, boolean queueSort, boolean preferExact, int nThreads, boolean useInt,
				Debug debug, int timeoutMilliseconds, int maximumNumberOfStates, int costUpperBound,
				boolean partiallyOrderEvents) {
			super(Algorithm.ASTAR, moveSort, queueSort, preferExact, nThreads, useInt, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, false, 0, 1);
		}

		public AStar(boolean moveSort, boolean queueSort, boolean preferExact, int nThreads, boolean useInt,
				Debug debug, int timeoutMilliseconds, int maximumNumberOfStates, int costUpperBound,
				boolean partiallyOrderEvents, int maxReducedSequenceLength) {
			super(Algorithm.ASTAR, moveSort, queueSort, preferExact, nThreads, useInt, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, false, 0, maxReducedSequenceLength);
		}
	}

	public final static class IncrementalAStar extends ReplayerParameters {
		public IncrementalAStar() {
			super(Algorithm.INCREMENTALASTAR, false, true, true,
					Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false, Debug.NONE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, Integer.MAX_VALUE, false, false, 0, 1);
		}

		public IncrementalAStar(Debug debug) {
			super(Algorithm.INCREMENTALASTAR, false, true, true,
					Math.max(1, Runtime.getRuntime().availableProcessors() / 2), false, debug, Integer.MAX_VALUE,
					Integer.MAX_VALUE, Integer.MAX_VALUE, false, false, 0, 1);
		}

		public IncrementalAStar(boolean moveSort, int nThreads, boolean useInt, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, int costUpperBound, boolean partiallyOrderEvents,
				boolean preProcessUsingPlaceBasedConstraints) {
			super(Algorithm.INCREMENTALASTAR, moveSort, true, true, nThreads, useInt, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, preProcessUsingPlaceBasedConstraints,
					0, 1);
		}

		public IncrementalAStar(boolean moveSort, int nThreads, boolean useInt, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, int costUpperBound, boolean partiallyOrderEvents, int initialSplits) {
			super(Algorithm.INCREMENTALASTAR, moveSort, true, true, nThreads, useInt, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, false, initialSplits, 1);
		}

		public IncrementalAStar(boolean moveSort, int nThreads, boolean useInt, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, int costUpperBound, boolean partiallyOrderEvents,
				boolean preProcessUsingPlaceBasedConstraints, int initialSplits, int maxReducedSequenceLength) {
			super(Algorithm.INCREMENTALASTAR, moveSort, true, true, nThreads, useInt, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, preProcessUsingPlaceBasedConstraints,
					initialSplits, maxReducedSequenceLength);
		}

	}

	public final static class Dijkstra extends ReplayerParameters {
		public Dijkstra() {
			super(Algorithm.DIJKSTRA, false, true, true, 1, false, Debug.NONE, Integer.MAX_VALUE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false, false, 0, 1);
		}

		public Dijkstra(Debug debug) {
			super(Algorithm.DIJKSTRA, false, true, true, 1, false, debug, Integer.MAX_VALUE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false, false, 0, 1);
		}

		public Dijkstra(boolean moveSort, boolean queueSort, int nThreads, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, int costUpperBound, boolean partiallyOrderEvents) {
			super(Algorithm.DIJKSTRA, moveSort, queueSort, true, nThreads, false, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, false, 0, 1);
		}

		public Dijkstra(boolean moveSort, boolean queueSort, int nThreads, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, int costUpperBound, boolean partiallyOrderEvents,
				int maxReducedSequenceLength) {
			super(Algorithm.DIJKSTRA, moveSort, queueSort, true, nThreads, false, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, false, 0, maxReducedSequenceLength);
		}
	}

	public final static class Full extends ReplayerParameters {
		public Full() {
			super(Algorithm.FULL, false, true, true, 1, false, Debug.NONE, Integer.MAX_VALUE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false, false, 0, 1);
		}

		public Full(Debug debug) {
			super(Algorithm.FULL, false, true, true, 1, false, debug, Integer.MAX_VALUE, Integer.MAX_VALUE,
					Integer.MAX_VALUE, false, false, 0, 1);
		}

		public Full(boolean moveSort, boolean queueSort, int nThreads, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, int costUpperBound, boolean partiallyOrderEvents) {
			super(Algorithm.FULL, moveSort, queueSort, true, nThreads, false, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, false, 0, 1);
		}

		public Full(boolean moveSort, boolean queueSort, int nThreads, Debug debug, int timeoutMilliseconds,
				int maximumNumberOfStates, int costUpperBound, boolean partiallyOrderEvents,
				int maxReducedSequenceLength) {
			super(Algorithm.FULL, moveSort, queueSort, true, nThreads, false, debug, timeoutMilliseconds,
					maximumNumberOfStates, costUpperBound, partiallyOrderEvents, false, 0, maxReducedSequenceLength);
		}
	}

}