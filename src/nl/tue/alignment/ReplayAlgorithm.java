package nl.tue.alignment;

import java.util.Arrays;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.util.HashBackedPriorityQueue;
import nl.tue.alignment.util.SortedHashBackedPriorityQueue;
import nl.tue.alignment.util.VisitedHashSet;

/**
 * The replay algorithm implements a replayer using the AStar skeleton. It
 * leaves implementations of the heuristic to implementing subclasses.
 * 
 * A few assumptions are made: <br />
 * 
 * Places can at most hold 3 tokens. If three tokens are in a place, the
 * preceding transitions are not enabled. If this situation is encountered, a
 * flag is set in the result Utils.ENABLINGBLOCKEDBYOUTPUT.
 * 
 * The cost function does not exceed 16777215. Since only 3 bytes are used to
 * keep it, the value of the cost function cannot exceed this limit. If this
 * happens, the algorithm ignores it, but sets the flag
 * Utils.COSTFUNCTIONOVERFLOW
 * 
 * The heuristic function does not exceed 16777214. Since only 3 bytes are used
 * to keep it and one value is reserved for infinity, the value of the heuristic
 * function cannot exceed this limit. If this happens, the algorithm sets the
 * flag Utils.HEURISTICFUNCTIONOVERFLOW and continues with the maximum heuristic
 * allowed.
 * 
 * Because of the maximum value of the cost function, it is wise not to set too
 * high values for the costs of firing certain transitions.
 */

/*- 
 * Internally, information is stored in five arrays:
 * byte[][] markingLo; 
 * byte[][] markingHi; 
 * int[][] ptl_g; 
 * int[][] e_pth_h;
 * int[][] c_p;
 * 
 * These arrays are double arrays, i.e. the used memory is split up into smaller blocks,
 * which allows for more efficient storage in memory, as well as synchronization on blocks
 * in future multi threaded versions.
 * 
 * markingLo and markingHi store the markings reached. Each marking is represented by 
 * two arrays of bytes (a low and high) array. Within these, each place has 1 bit.
 * i.e. [1000 1000][000.....] as low bytes and [1000 0000][010. ....] as high bytes 
 * together represent the marking with 3 tokens in place 0, 1 token in place 4 and 2 
 * tokens in place 9, assuming 11 places in the model (hence the 5 unused trailng bits). 
 * These bytes are serialized into markingLo and markingHi.
 * 
 * The array ptl_g stores the low bits of the preceding transition relation in the first byte and
 * the remaining 3 bytes are used to store the value for g (unsigned, i.e. assumed maximum 
 * is 16777215).
 * 
 * The array e_pth_h consists of three parts. The highest bit is a flag indicating if the 
 * stored heuristic is exact or estimated. Then, 7 bits are used to store the high bits 
 * of the preceding transition relation. The remaining 3 bytes are used to store the estimated 
 * value of h (Unsigned, assumed maximum is 16777214, since 16777215 stands for infinite).
 * 
 * The ptl relation is composed of the 8 low bits from ptl_g and the 7 high bits from pth_h. 
 * Combined with a highest bit equal to 0 they form a short value which is interpreted signed, 
 * i.e. maximum 32767.
 * 
 * The array c_p uses the highest bit to indicate that a marking is in the closed set. The
 * remaining 31 bits are used to store the predecessor index.
 * 
 * @author bfvdonge
 *
 */
public abstract class ReplayAlgorithm {

	public static enum Debug {
		DOT {
			@Override
			public void writeMarkingReached(ReplayAlgorithm algorithm, int marking, String extra) {
				int heur = algorithm.getHScore(marking);
				StringBuilder b = new StringBuilder();
				b.append("m");
				b.append(marking);
				b.append(" [label=<");
				if (algorithm.isClosed(marking)) {
					b.append("(m");
					b.append(marking);
					b.append(")");
				} else {
					b.append("m");
					b.append(marking);
				}
				b.append("<BR/>");
				b.append(Utils.asBag(algorithm.getMarking(marking), algorithm.net));
				b.append("<BR/>g=");
				b.append(algorithm.getGScore(marking));
				b.append(",");
				b.append(algorithm.hasExactHeuristic(marking) ? "h" : "~h");
				b.append("=");
				b.append((heur == HEURISTICINFINITE ? "inf" : heur));
				b.append(">");
				if (!extra.isEmpty()) {
					b.append(",");
					b.append(extra);
				}
				b.append("];");
				System.out.println(b.toString());
			}

			@Override
			public void writeEdgeTraversed(ReplayAlgorithm algorithm, int fromMarking, short transition, int toMarking,
					String extra) {
				StringBuilder b = new StringBuilder();
				b.append("m");
				b.append(fromMarking);
				b.append(" -> m");
				b.append(toMarking);
				b.append(" [");
				b.append("label=<");
				//				b.append("t");
				//				b.append(transition);
				//				b.append("<br/>");
				b.append(algorithm.net.getTransitionLabel(transition));
				b.append("<br/>");
				b.append(algorithm.net.getCost(transition));
				b.append(">");
				if (!extra.isEmpty()) {
					b.append(",");
					b.append(extra);
				}
				b.append("];");
				System.out.println(b.toString());
			}
		}, //
		NORMAL {

		}, //
		NONE;

		private static String EMPTY = "";

		public void writeEdgeTraversed(ReplayAlgorithm algorithm, int fromMarking, short transition, int toMarking,
				String extra) {
		}

		public void writeMarkingReached(ReplayAlgorithm algorithm, int marking, String extra) {
		}

		public void writeEdgeTraversed(ReplayAlgorithm algorithm, int fromMarking, short transition, int toMarking) {
			this.writeEdgeTraversed(algorithm, fromMarking, transition, toMarking, EMPTY);
		}

		public void writeMarkingReached(ReplayAlgorithm algorithm, int marking) {
			this.writeMarkingReached(algorithm, marking, EMPTY);
		}

		public void writeDebugInfo(Debug db, String s) {
			if (this == db) {
				System.out.println(s);
			}
		}
	}

	private static final int PTRANSLOMASK = 0b11111111000000000000000000000000;
	private static final int GMASK = 0b00000000111111111111111111111111;

	private static final int EXACTMASK = 0b10000000000000000000000000000000;
	private static final int PTRANSHIMASK = 0b01111111000000000000000000000000;
	private static final int HMASK = 0b00000000111111111111111111111111;

	private static final int CLOSEDMASK = 0b10000000000000000000000000000000;
	private static final int PMASK = 0b01111111111111111111111111111111;

	protected static final int NOPREDECESSOR = 0b01111111111111111111111111111111;

	protected static int HEURISTICINFINITE = HMASK;

	/**
	 * Stores the number of bytes reserved for input and output array. A marking
	 * takes twice this number of bytes.
	 */
	protected final int bm;

	/**
	 * Stores the blockSize as a power of 2
	 */
	protected final int blockSize;

	/**
	 * Stores the number of trailing 0's in the blockSize.
	 */
	protected final int blockBit;

	/**
	 * equals blockSize-1
	 */
	protected final int blockMask;

	/**
	 * Stores the last block in use
	 */
	protected int block = -1;

	/**
	 * Stores the first new index in current block
	 */
	protected int indexInBlock;

	//	/**
	//	 * Stores the low bits of the markings marking m is stored at
	//	 * markingLo[bm*m]..markingLo[bm*m+bm-1] and
	//	 * markingHi[bm*m]..markingHi[bm*m+bm-1]
	//	 * 
	//	 * if a place p is marked with 0 tokens: markingLo[bm*m + p/8] & (FLAG >>>
	//	 * (p%8)) == 0 markingHi[bm*m + p/8] & (1 >>> (p%8)) == 0
	//	 * 
	//	 * if a place p is marked with 1 token: markingLo[bm*m + p/8] & (FLAG >>>
	//	 * (p%8)) == 1 markingHi[bm*m + p/8] & (1 >>> (p%8)) == 0
	//	 * 
	//	 * if a place p is marked with 2 tokens: markingLo[bm*m + p/8] & (FLAG >>>
	//	 * (p%8)) == 0 markingHi[bm*m + p/8] & (1 >>> (p%8)) == 1
	//	 * 
	//	 * if a place p is marked with 3 tokens: markingLo[bm*m + p/8] & (FLAG >>>
	//	 * (p%8)) == 1 markingHi[bm*m + p/8] & (1 >>> (p%8)) == 1
	//	 * 
	//	 * Note (p%8) = (p&7)
	//	 */
	//	protected byte[][] markingLo = new byte[0][];
	//	protected byte[][] markingHi = new byte[0][];;

	/**
	 * Stores the true distance to reach a marking in the last 3 bytes. For marking
	 * m, g[m]&GMASK is the shortest distance so far to reach m
	 * 
	 * Also stores part of the predecessor transition (8 lowest bits of 15 total)
	 * 
	 */
	protected int[][] ptl_g = new int[0][];

	/**
	 * Stores the heuristic distance to reach the final marking in the last 3 bytes.
	 * For marking m, h[m] & HMASK is an underestimate of the shortest distance to
	 * reach the final marking from m
	 * 
	 * Also stores the estimated heuristic flag. Marking m is in the estimated
	 * heuristic set if h[m]& ESTIMATEDFLAG == ESTIMATEDFLAG.
	 * 
	 * Also stores part of the predecessor transition (7 lowest bits of 14 total)
	 * 
	 */
	protected int[][] e_pth_h = new int[0][];

	/**
	 * Stores the predecessor relation for which the distance so far is minimal.
	 * 
	 * Also stores the closed set in the highest bit of each element.
	 * 
	 * For marking m, marking p[m] is the predecessor
	 * 
	 * For marking m, it is in the closed set if (p[m] & CLOSEDMASK) == CLOSEDMASK
	 */
	protected int[][] c_p = new int[0][];

	/**
	 * Stores the closed set
	 */
	protected final VisitedSet visited;

	/**
	 * The synchronous product under investigation
	 */
	protected final SyncProduct net;

	/**
	 * Stores the open set as a priority queue
	 */
	protected final Queue queue;

	/**
	 * Indicate if moves should be considered totally ordered.
	 */
	protected boolean moveSorting;

	/**
	 * Stores the selected debug level
	 */
	protected final Debug debug;

	/**
	 * Flag indicating if exact solutions should be kept separate
	 */
	protected final boolean preferExact;

	protected int pollActions;
	protected int closedActions;
	protected int queueActions;
	protected int edgesTraversed;
	protected int markingsReached;
	protected int heuristicsComputed;
	protected int heuristicsEstimated;
	protected int heuristicsDerived;
	protected int alignmentLength;
	protected int alignmentCost;
	protected int alignmentResult;
	protected int setupTime;
	protected int runTime;

	protected long startConstructor;
	protected short numPlaces;

	public ReplayAlgorithm(SyncProduct product, boolean moveSorting, boolean queueSorting, boolean preferExact) {
		this(product, moveSorting, queueSorting, preferExact, Debug.NONE);
	}

	public ReplayAlgorithm(SyncProduct product, boolean moveSorting, boolean queueSorting, boolean preferExact,
			Debug debug) {
		this.preferExact = preferExact;
		this.debug = debug;
		startConstructor = System.nanoTime();

		this.numPlaces = product.numPlaces();
		this.net = product;
		this.moveSorting = moveSorting;
		// compute how many bytes are needed to store 1 bit per place
		this.bm = 1 + (product.numPlaces() - 1) / 8;
		// set the blocksize such that each array fits within the
		// DEFAULTBLOCKSIZE, but it should be able to hold at least 2 markings
		int blocks = Math.max(Utils.DEFAULTBLOCKSIZE / bm, 2 * bm);
		int s = 1;
		int t = 0;
		while (blocks > s) {
			s = s << 1;
			t++;
		}
		this.blockBit = t;
		this.blockSize = s;
		this.blockMask = blockSize - 1;

		this.visited = new VisitedHashSet(this, Utils.DEFAULTVISITEDSIZE);
		if (queueSorting) {
			if (preferExact) {
				this.queue = new SortedHashBackedPriorityQueue(this, Utils.DEFAULTQUEUESIZE, Integer.MAX_VALUE, true);
			} else {
				this.queue = new SortedHashBackedPriorityQueue(this, Utils.DEFAULTQUEUESIZE, Integer.MAX_VALUE, false);
			}
		} else {
			this.queue = new HashBackedPriorityQueue(this, Utils.DEFAULTQUEUESIZE, Integer.MAX_VALUE);
		}

		// Array used internally for firing transitions.
		newMarking = new byte[product.numPlaces()];

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);
	}

	public TObjectIntMap<Utils.Statistic> getStatistics() {
		TObjectIntMap<Utils.Statistic> map = new TObjectIntHashMap<>(20);
		map.put(Utils.Statistic.POLLACTIONS, pollActions);
		map.put(Utils.Statistic.CLOSEDACTIONS, closedActions);
		map.put(Utils.Statistic.QUEUEACTIONS, queueActions);
		map.put(Utils.Statistic.EDGESTRAVERSED, edgesTraversed);
		map.put(Utils.Statistic.MARKINGSREACHED, markingsReached);
		map.put(Utils.Statistic.HEURISTICSCOMPUTED, heuristicsComputed);
		map.put(Utils.Statistic.HEURISTICSDERIVED, heuristicsDerived);
		map.put(Utils.Statistic.HEURISTICSESTIMATED, heuristicsEstimated);
		map.put(Utils.Statistic.ALIGNMENTLENGTH, alignmentLength);
		map.put(Utils.Statistic.COST, alignmentCost);
		map.put(Utils.Statistic.RELIABLE, alignmentResult);
		map.put(Utils.Statistic.RUNTIME, runTime);
		map.put(Utils.Statistic.SETUPTIME, setupTime);
		map.put(Utils.Statistic.TOTALTIME, setupTime + runTime);
		map.put(Utils.Statistic.MAXQUEUELENGTH, queue.maxSize());
		map.put(Utils.Statistic.MAXQUEUECAPACITY, queue.maxCapacity());
		map.put(Utils.Statistic.VISITEDSETCAPACITY, visited.capacity());

		//TODO: Count the bytes in the hashtable backing the queue!
		map.put(Utils.Statistic.MEMORYUSED, (int) (getEstimatedMemorySize() / 1024));
		return map;
	}

	protected long getEstimatedMemorySize() {
		// g holds         4 + length * 8 + block * (4 + blockSize * 4) bytes;
		// h holds         4 + length * 8 + block * (4 + blockSize * 4) bytes;
		// p holds         4 + length * 8 + block * (4 + blockSize * 4) bytes;

		// each array has 4 bytes overhead for storing the size
		long val = 3 * 4 + 3 * ptl_g.length * 8 + 3 * block * (4 + blockSize * 4);

		// count the capacity of the queue
		val += queue.maxBytesUsed();
		// count the capacity of the visistedSet
		val += 4 + visited.capacity() * 4;

		return val;
	}

	public short[] run() {
		//		short[] trans = new short[net.numTransitions()];
		//		for (short t = net.numTransitions(); t-- > 0;) {
		//			trans[t] = t;
		//		}
		//		Utils.shuffleArray(trans, new Random());

		debug.writeDebugInfo(Debug.DOT, "Digraph D {");
		try {
			growArrays();

			pollActions = 0;
			closedActions = 0;
			queueActions = 1;
			edgesTraversed = 0;
			markingsReached = 0;
			heuristicsComputed = 0;
			heuristicsEstimated = 0;
			heuristicsDerived = 0;
			alignmentLength = 0;
			alignmentCost = 0;
			alignmentResult = 0;
			runTime = 0;

			long start = System.nanoTime();

			// get the initial marking
			byte[] initialMarking = net.getInitialMarking();
			// add to the set of markings
			int pos = addNewMarking(initialMarking);
			int b = pos >>> blockBit;
			int i = pos & blockMask;
			// set predecessor to null
			setPredecessor(b, i, NOPREDECESSOR);
			setPredecessorTransition(b, i, (short) 0);
			setGScore(b, i, 0);
			int heuristic = getExactHeuristic(0, initialMarking, b, i);
			setHScore(b, i, heuristic, true);

			queue.add(0);

			while (!queue.isEmpty()) {

				// get the most promising marking m
				int m = queue.poll();
				pollActions++;

				int bm = m >>> blockBit;
				int im = m & blockMask;

				if (isFinal(m)) {
					return handleFinalMarkingReached(start, m);
				}

				// add m to the closed set
				setClosed(bm, im);
				closedActions++;

				if (debug == Debug.NORMAL && closedActions % 10000 == 0) {
					writeStatus();
				}

				byte[] marking_m = getMarking(bm, im);
				if (!hasExactHeuristic(bm, im)) {
					// compute the exact heuristic
					heuristic = getExactHeuristic(m, marking_m, bm, im);

					if (heuristic == HEURISTICINFINITE) {
						// marking from which final marking is unreachable
						// ignore state and continue

						continue;
					} else if (heuristic > getHScore(bm, im)) {
						// if the heuristic is higher, set the score
						setHScore(bm, im, heuristic, true);
						// push the head of the queue down

						queue.add(m);
						queueActions++;
						continue;
					}
					// set the score to exact score
					setHScore(bm, im, heuristic, true);
					// continue with this marking
				}

				//			System.out.println("m" + m + " [" + Utils.print(getMarking(m), net.numPlaces()) + "];");

				// iterate over all transitions
				for (short t = 0; t < net.numTransitions(); t++) {
					//				for (short t = net.numTransitions(); t-- > 0;) {
					//				for (short t : trans) {

					// check for enabling
					if (isEnabled(marking_m, t, bm, im)) {
						edgesTraversed++;

						// t is allowed to fire.
						byte[] n_array = fire(marking_m, t, bm, im);

						// check if n already reached before
						int n = visited.add(n_array, block * blockSize + indexInBlock);
						// adding the marking to the algorithm is handled by the VisitedSet.

						//					System.out.println("   Fire " + t + ": " + Utils.print(getMarking(n), net.numPlaces()));

						int bn = n >>> blockBit;
						int in = n & blockMask;
						if (!isClosed(bn, in)) {
							boolean add = false;

							// n is a fresh marking, not in the closed set
							// compute the F score on this path
							int tmpG = getGScore(bm, im) + net.getCost(t);
							// update path if GScore equal and transtion lower order
							if (moveSorting && tmpG == getGScore(bn, in) && getPredecessorTransition(bn, in) > t) {
								// when sorting moves, enabled transitions are bound to the highest incoming
								// transition. For equal G score, the highest transition going into a marking is
								// therefore preferred.
								debug.writeEdgeTraversed(this, m, t, n, "color=green");
								setPredecessor(bn, in, m);
								setPredecessorTransition(bn, in, t);

								add = !preferExact || hasExactHeuristic(bn, in);
							}

							if (tmpG < getGScore(bn, in)) {
								debug.writeEdgeTraversed(this, m, t, n);

								// found a shorter path to n.
								setGScore(bn, in, tmpG);

								// set predecessor
								setPredecessor(bn, in, m);
								setPredecessorTransition(bn, in, t);

								if (!hasExactHeuristic(bn, in)) {
									// estimate is not exact, so derive a new estimate (note that h cannot decrease here)
									deriveOrEstimateHValue(m, bm, im, t, n, bn, in);
								}

								// update position of n in the queue
								queue.add(n);
								queueActions++;

							} else if (preferExact && !hasExactHeuristic(bn, in)) {
								//tmpG >= getGScore(n), i.e. we reached state n through a longer path.

								// G shore might not be an improvement, but see if we can derive the 
								// H score. //TODO: Currently only if preferExact is true. This can 
								// be changed, but requires the priority queue to support demotion
								// of elements instead of only promotion, since H can only increase.
								deriveOrEstimateHValue(m, bm, im, t, n, bn, in);
								if (hasExactHeuristic(bn, in)) {
									debug.writeEdgeTraversed(this, m, t, n, "color=blue");
									// marking is now exact and was not before. 
									// if not in the queue yet (preferExact) queue now.
									assert !queue.contains(n);
									queue.add(n);
									queueActions++;
								}
							} else {
								debug.writeEdgeTraversed(this, m, t, n, "color=gray");
							}
						} else {
							debug.writeEdgeTraversed(this, m, t, n, "color=gray");
						}
					}
				}
			}
			alignmentResult &= ~Utils.OPTIMALALIGNMENT;
			alignmentResult |= Utils.FAILEDALIGNMENT;
			runTime = (int) ((System.nanoTime() - start) / 1000);
			return null;
		} finally {
			terminateRun();

			if (debug == Debug.DOT) {
				writeEndOfAlignmentDot();
			}
			if (debug == Debug.NORMAL) {
				writeEndOfAlignmentNormal();
			}

		}

	}

	protected short[] handleFinalMarkingReached(long startTime, int marking) {
		// Final marking reached.
		int n = getPredecessor(marking);
		int m2 = marking;
		short t;
		while (n != NOPREDECESSOR) {
			t = getPredecessorTransition(m2);

			debug.writeEdgeTraversed(this, n, t, m2, "color=red,fontcolor=red");
			alignmentLength++;
			alignmentCost += net.getCost(t);
			m2 = n;
			n = getPredecessor(n);
		}
		short[] alignment = new short[alignmentLength];
		n = getPredecessor(marking);
		m2 = marking;
		int l = alignmentLength;
		while (n != NOPREDECESSOR) {
			t = getPredecessorTransition(m2);
			alignment[--l] = t;
			m2 = n;
			n = getPredecessor(n);
		}

		alignmentResult |= Utils.OPTIMALALIGNMENT;
		runTime = (int) ((System.nanoTime() - startTime) / 1000);

		return alignment;
	}

	protected void writeEndOfAlignmentNormal() {
		TObjectIntMap<Statistic> map = getStatistics();
		for (Statistic s : Statistic.values()) {
			debug.writeDebugInfo(Debug.NORMAL, s + ": " + map.get(s));
		}
	}

	protected void writeEndOfAlignmentDot() {
		TObjectIntMap<Statistic> map = getStatistics();
		for (int m = 0; m < markingsReached; m++) {
			debug.writeMarkingReached(this, m);
		}
		StringBuilder b = new StringBuilder();
		b.append("info [shape=plaintext,label=<");
		for (Statistic s : Statistic.values()) {
			b.append(s);
			b.append(": ");
			b.append(map.get(s));
			b.append("<br/>");
		}
		b.append(">];");

		debug.writeDebugInfo(Debug.DOT, b.toString());
		debug.writeDebugInfo(Debug.DOT, "}");
	}

	protected void terminateRun() {
	}

	protected abstract void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to,
			int toBlock, int toIndex);

	protected abstract boolean isFinal(int marking);

	// Used internally in firing.
	private transient byte[] newMarking;

	protected byte[] fire(byte[] fromMarking, short transition, int block, int index) {
		// fire transition t in marking stored at block, index
		// First consumption:
		short[] input = net.getInput(transition);
		short[] output = net.getOutput(transition);
		System.arraycopy(fromMarking, 0, newMarking, 0, numPlaces);

		for (int i = input.length; i-- > 0;) {
			newMarking[input[i]]--;
		}
		for (int i = output.length; i-- > 0;) {
			newMarking[output[i]]++;
		}
		//		
		//		for (int i = bm; i-- > 0;) {
		//			newMarking[i] = (byte) ((markingLo[block][bm * index + i] ^ input[i]) & 0xFF);
		//			byte tmp = (byte) ((newMarking[i] & input[i]) & 0xFF);
		//			newMarking[bm + i] = (byte) ((markingHi[block][bm * index + i] ^ tmp) & 0xFF);
		//		}
		//		// now production
		//		for (int i = bm; i-- > 0;) {
		//			byte tmp = (byte) ((newMarking[i] & output[i]) & 0xFF);
		//			newMarking[i] = (byte) ((newMarking[i] ^ output[i]) & 0xFF);
		//			newMarking[bm + i] = (byte) ((newMarking[bm + i] ^ tmp) & 0xFF);
		//		}

		return newMarking;
	}

	protected void unfire(byte[] marking, short transition) {
		// fire transition t in marking stored at block, index
		// First consumption:
		short[] input = net.getInput(transition);
		short[] output = net.getOutput(transition);

		for (int i = input.length; i-- > 0;) {
			marking[input[i]]--;
		}
		for (int i = output.length; i-- > 0;) {
			marking[output[i]]++;
		}
	}

	protected boolean isEnabled(byte[] marking, short transition, int block, int index) {
		// check enablement on the tokens and on the predecessor
		short preTransition = getPredecessorTransition(block, index);
		if (!moveSorting || preTransition <= transition || hasPlaceBetween(preTransition, transition)) {
			// allow firing only if there is a place between or if total order
			// is
			// respected
			short[] input = net.getInput(transition);
			for (int i = input.length; i-- > 0;) {
				if (marking[input[i]] < 1) {
					return false;
				}
			}
			short[] output = net.getOutput(transition);
			for (int i = output.length; i-- > 0;) {
				if (marking[output[i]] > 2) {
					alignmentResult |= Utils.ENABLINGBLOCKEDBYOUTPUT;
					return false;
				}
			}

			//			for (int i = bm; i-- > 0;) {
			//				// ((markingLo OR markingHi) AND input) should be input.
			//				if (((markingLo[block][bm * index + i] | markingHi[block][bm * index + i]) & input[i]) != input[i]) {
			//					return false;
			//				}
			//			}
			//			// Firing semantics do not allow to produce more than 3 tokens
			//			// in a place ((markingLo AND markingHi) AND output) should be 0.
			//			byte[] output = net.getOutput(transition);
			//			for (int i = bm; i-- > 0;) {
			//				if (((markingLo[block][bm * index + i] & markingHi[block][bm * index + i]) & output[i]) != 0) {
			//					// if violated, signal in alignmentResult and continue
			//					alignmentResult |= Utils.ENABLINGBLOCKEDBYOUTPUT;
			//					return false;
			//				}
			//			}
			return true;
		} else {
			// not allowed to fire
			return false;
		}
	}

	/**
	 * returns true if there is a place common in the output set of transitionFrom
	 * and the input set of transitionTo
	 * 
	 * @param transitionFrom
	 * @param transitionTo
	 * @return
	 */
	public boolean hasPlaceBetween(short preTransition, short transition) {
		short[] input = net.getInput(transition);
		short[] output = net.getOutput(preTransition);
		// Note, input and output are SORTED LISTS.
		int i = 0, j = 0;
		while (i < input.length && j < output.length) {
			if (input[i] < output[j]) {
				i++;
			} else if (input[i] > output[j]) {
				j++;
			} else {
				assert input[i] == output[j];
				return true;
			}
		}
		return false;
	}

	private byte[] getMarking(int block, int index) {
		byte[] marking = new byte[numPlaces];
		// add initial marking
		System.arraycopy(net.getInitialMarking(), 0, marking, 0, numPlaces);
		// unfire all transitions in the sequence back
		short t = getPredecessorTransition(block, index);
		int m = getPredecessor(block, index);
		while (m != NOPREDECESSOR) {
			block = m >>> blockBit;
			index = m & blockMask;
			unfire(marking, t);
			m = getPredecessor(block, index);
			t = getPredecessorTransition(block, index);
		}
		return marking;
	}

	public byte[] getMarking(int marking) {
		return getMarking(marking >>> blockBit, marking & blockMask);
	}

	public int addNewMarking(byte[] marking) {
		// allocate space for writing marking information in the block
		markingsReached++;
		int pos = block * blockSize + indexInBlock;
		//		System.arraycopy(marking, 0, markingLo[block], bm * indexInBlock, bm);
		//		System.arraycopy(marking, bm, markingHi[block], bm * indexInBlock, bm);
		indexInBlock++;
		if (indexInBlock >= blockSize) {
			// write pointer moved over blockSize,
			// allocate a new block
			growArrays();
		}
		return pos;
	}

	protected void writeStatus() {
		debug.writeDebugInfo(Debug.NORMAL, "Markings reached:   " + String.format("%,d", markingsReached));
		debug.writeDebugInfo(Debug.NORMAL, "   FScore head:     " + getFScore(queue.peek()) + " = G: "
				+ getGScore(queue.peek()) + " + H: " + getHScore(queue.peek()));
		debug.writeDebugInfo(Debug.NORMAL, "   Queue size:      " + queue.size());
		debug.writeDebugInfo(Debug.NORMAL, "   Queue actions:   " + queueActions);
		debug.writeDebugInfo(Debug.NORMAL, "   Heuristics compu:" + heuristicsComputed);
		debug.writeDebugInfo(Debug.NORMAL, "   Estimated memory:" + String.format("%,d", getEstimatedMemorySize()));
	}

	/**
	 * Grow the internal array structure. Method should be considered synchronized
	 * as it should not be executed in parallel.
	 */
	protected void growArrays() {
		if (block + 1 >= ptl_g.length) {
			int newLength = ptl_g.length < 64 ? ptl_g.length * 2 : (ptl_g.length * 3) / 2;
			if (newLength <= block + 1) {
				newLength = block + 2;
			}
			//			markingLo = Arrays.copyOf(markingLo, newLength);
			//			markingHi = Arrays.copyOf(markingHi, newLength);
			ptl_g = Arrays.copyOf(ptl_g, newLength);
			e_pth_h = Arrays.copyOf(e_pth_h, newLength);
			c_p = Arrays.copyOf(c_p, newLength);
		}
		// increase the block pointer
		block++;
		// reset the index in block
		indexInBlock = 0;

		//		// markingLo holds blocksize markings of bm bytes each
		//		markingLo[block] = new byte[blockSize * bm];
		//		// markingHi holds blocksize markings of bm bytes each
		//		markingHi[block] = new byte[blockSize * bm];
		// g holds blocksize costs
		ptl_g[block] = new int[blockSize];
		Arrays.fill(ptl_g[block], GMASK);
		// h holds blocksize estimates
		e_pth_h[block] = new int[blockSize];
		// p holds blocksize predecessors
		c_p[block] = new int[blockSize];

	}

	/**
	 * Returns the f score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getFScore(int marking) {
		return getFScore(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the f score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getFScore(int block, int index) {
		return getGScore(block, index) + getHScore(block, index);
	}

	/**
	 * Returns the g score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getGScore(int marking) {
		return getGScore(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getGScore(int block, int index) {
		return ptl_g[block][index] & GMASK;
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setGScore(int block, int index, int score) {
		// overwrite the last three bytes of the score.
		ptl_g[block][index] &= ~GMASK;
		if ((score & GMASK) != score) {
			alignmentResult |= Utils.COSTFUNCTIONOVERFLOW;
			ptl_g[block][index] |= score & GMASK;
		} else {
			ptl_g[block][index] |= score;
		}
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setGScore(int marking, int score) {
		setGScore(marking >>> blockBit, marking & blockMask, score);
	}

	/**
	 * Returns the h score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getHScore(int marking) {
		return getHScore(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the h score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getHScore(int block, int index) {
		return e_pth_h[block][index] & HMASK;
	}

	/**
	 * set the h score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setHScore(int block, int index, int score, boolean isExact) {
		assert (score & HMASK) == score;
		// overwrite the last three bytes of the score.
		e_pth_h[block][index] &= ~HMASK; // reset to 0
		e_pth_h[block][index] |= score; // set score
		if (isExact) {
			e_pth_h[block][index] |= EXACTMASK; // set exactFlag
		} else {
			e_pth_h[block][index] &= ~EXACTMASK; // clear exactFlag

		}
	}

	/**
	 * Set the h score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setHScore(int marking, int score, boolean isExact) {
		setHScore(marking >>> blockBit, marking & blockMask, score, isExact);
	}

	/**
	 * Returns the predecessor for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getPredecessor(int marking) {
		return getPredecessor(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the predecessor for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getPredecessor(int block, int index) {
		return c_p[block][index] & PMASK;
	}

	/**
	 * Sets the predecessor for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setPredecessor(int block, int index, int predecessorMarking) {
		c_p[block][index] &= ~PMASK;
		c_p[block][index] |= predecessorMarking & PMASK;
	}

	/**
	 * Sets the predecessor for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setPredecessor(int marking, int predecessorMarking) {
		setPredecessor(marking >>> blockBit, marking & blockMask, predecessorMarking);
	}

	/**
	 * Returns the predecessor transition for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public short getPredecessorTransition(int marking) {
		return getPredecessorTransition(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the predecessor transition for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public short getPredecessorTransition(int block, int index) {
		return (short) (((ptl_g[block][index] & PTRANSLOMASK) >>> 24) | // low bits from g
				((e_pth_h[block][index] & PTRANSHIMASK) >>> 16)); // high bits from h
	}

	/**
	 * Sets the predecessor transition for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setPredecessorTransition(int block, int index, short transition) {
		ptl_g[block][index] &= ~PTRANSLOMASK; //clear pt bits
		ptl_g[block][index] |= (transition << 24) & PTRANSLOMASK; //store the high bits of transition
		e_pth_h[block][index] &= ~PTRANSHIMASK; //clear pt bits
		e_pth_h[block][index] |= (transition << 16) & PTRANSHIMASK; //store the low bits of transition
	}

	/**
	 * Sets the predecessor transition for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setPredecessorTransition(int marking, short transition) {
		setPredecessorTransition(marking >>> blockBit, marking & blockMask, transition);
	}

	/**
	 * Returns true if marking is in the closed set
	 * 
	 * @param marking
	 * @return
	 */
	public boolean isClosed(int marking) {
		return isClosed(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public boolean isClosed(int block, int index) {
		return (c_p[block][index] & CLOSEDMASK) == CLOSEDMASK;
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setClosed(int block, int index) {
		c_p[block][index] |= CLOSEDMASK;
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setClosed(int marking) {
		setClosed(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the h score for the given marking
	 * 
	 * @param marking
	 * @return
	 */
	public abstract int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex);

	/**
	 * Get the exact heuristic for a state that is currently estimated
	 * 
	 * @param marking
	 * @param markingBlock
	 * @param markingIndex
	 * @return
	 */
	protected int getExactHeuristicForEstimated(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		return getExactHeuristic(marking, markingArray, markingBlock, markingIndex);
	}

	/**
	 * returns true if the heuristic stored for the given marking is exact or an
	 * estimate.
	 * 
	 * @param marking
	 * @return
	 */
	public boolean hasExactHeuristic(int marking) {
		return hasExactHeuristic(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * returns true if the heuristic stored for the given marking is exact or an
	 * estimate.
	 * 
	 * @param marking
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public boolean hasExactHeuristic(int block, int index) {
		return (e_pth_h[block][index] & EXACTMASK) == EXACTMASK;
	}

	/**
	 * Checks equality of the stored marking1 to the given marking2.
	 * 
	 * marking2 is provided as an array of length 2*bm, where the first bm bytes
	 * provide the low bits and the second bm bytes provide the high bits.
	 * 
	 * @see SyncProduct.getInitialMarking();
	 * 
	 * @param marking1
	 * @param marking2
	 * @return
	 */
	public boolean equalMarking(int marking1, byte[] marking2) {
		return Arrays.equals(getMarking(marking1), marking2);
		//		int b = marking1 >>> blockBit;
		//		int i = marking1 & blockMask;
		//		return equalMarking(b, i, marking2);
	}

	//	protected boolean equalMarking(int block, int index, byte[] marking2) {
	//		
	//		for (int j = bm; j-- > 0;) {
	//			if (markingLo[block][bm * index + j] != marking2[j] || //
	//					markingHi[block][bm * index + j] != marking2[bm + j]) {
	//				return false;
	//			}
	//		}
	//		return true;
	//	}

	/**
	 * Returns the hashCode of a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int hashCode(int marking) {
		return hashCode(getMarking(marking));
		//		int b = marking >>> blockBit;
		//		int i = marking & blockMask;
		//		return hashCodeInternal(markingLo[b], bm * i, bm, markingHi[b], bm * i, bm);
	}

	/**
	 * Returns the hashCode of a stored marking which is provided as an array of
	 * length 2*bm, where the first bm bytes provide the low bits and the second bm
	 * bytes provide the high bits.
	 * 
	 * @see SyncProduct.getInitialMarking();
	 * @param marking
	 * @return
	 */
	public int hashCode(byte[] marking) {
		return Arrays.hashCode(marking);
		//		return hashCodeInternal(marking, 0, bm, marking, bm, bm);
	}

	/**
	 * Helper method to hash markings.
	 * 
	 * @return
	 */
	private int hashCodeInternal(byte[] l1, int s1, int n1, byte[] l2, int s2, int n2) {
		int result = 1;
		for (int j = s1 + n1; j-- > s1;) {
			result = 31 * result + l1[j];
		}
		for (int j = s2 + n2; j-- > s2;) {
			result = 31 * result + l2[j];
		}

		return result;
	}

}
