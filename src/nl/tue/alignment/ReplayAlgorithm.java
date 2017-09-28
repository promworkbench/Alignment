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

	//	private static final int PTRANSLOMASK = 0b11111111000000000000000000000000;

	//	private static final int PTRANSHIMASK = 0b01111111000000000000000000000000;

	protected static final long EXACTMASK = 0b1000000000000000000000000000000000000000000000000000000000000000L;
	protected static final long COMPUTINGMASK = 0b0100000000000000000000000000000000000000000000000000000000000000L;
	protected static final long HMASK = 0b0000000000000000000000000011111111111111111111111100000000000000L;
	protected static final int HSHIFT = 14;
	protected static final long GMASK = 0b0011111111111111111111111100000000000000000000000000000000000000L;
	protected static final int GSHIFT = 38;
	protected static final long PTMASK = 0b0000000000000000000000000000000000000000000000000011111111111111L;

	protected static final int CLOSEDMASK = 0b10000000000000000000000000000000;
	protected static final int PMASK = 0b01111111111111111111111111111111;

	protected static final int NOPREDECESSOR = PMASK;

	protected static int HEURISTICINFINITE = 0b00000000111111111111111111111111;

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
	private int indexInBlock;

	/**
	 * For each marking stores: 1 bit: whether it is estimated 24 bit: Value of
	 * g function 24 bit: Value of h function 15 bit: Predecessor transition
	 */
	protected long[][] e_g_h_pt = new long[0][];

	/**
	 * Stores the predecessor relation for which the distance so far is minimal.
	 * 
	 * Also stores the closed set in the highest bit of each element.
	 * 
	 * For marking m, marking p[m] is the predecessor
	 * 
	 * For marking m, it is in the closed set if (p[m] & CLOSEDMASK) ==
	 * CLOSEDMASK
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

		this.blockSize = Utils.DEFAULTBLOCKSIZE;
		this.blockMask = blockSize - 1;
		int bit = 1;
		int i = 0;
		while (bit < blockMask) {
			bit <<= 1;
			i++;
		}
		this.blockBit = i;

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
		firingMarking = new byte[product.numPlaces()];
		hashCodeMarking = new byte[product.numPlaces()];
		equalMarking = new byte[product.numPlaces()];

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
		// e_g_h_pt holds    4 + length * 8 + block * (4 + blockSize * 8) bytes;
		// c_p holds         4 + length * 8 + block * (4 + blockSize * 4) bytes;

		// each array has 4 bytes overhead for storing the size
		long val = 2 * 4 + 2 * c_p.length * 8 + block * (8 + blockSize * 12);

		// count the capacity of the queue
		val += queue.maxBytesUsed();
		// count the capacity of the visistedSet
		val += 4 + visited.capacity() * 4;

		return val;
	}

	public short[] run() throws Exception {
		return runReplayAlgorithm(System.nanoTime());
	}

	protected short[] runReplayAlgorithm(long startTime) {

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

			// get the initial marking
			byte[] initialMarking = net.getInitialMarking();
			// add to the set of markings
			int pos = addNewMarking(initialMarking);
			markingsReached++;

			int b = pos >>> blockBit;
			int i = pos & blockMask;
			// set predecessor to null
			setPredecessor(b, i, NOPREDECESSOR);
			setPredecessorTransition(b, i, (short) 0);
			setGScore(b, i, 0);

			int heuristic;
			//			System.out.println("Main waiting for " + b + "," + i);
			getLockForComputingEstimate(b, i);
			//			System.out.println("Main locking " + b + "," + i);
			try {
				heuristic = getExactHeuristic(0, initialMarking, b, i);
				setHScore(b, i, heuristic, true);
			} finally {
				releaseLockForComputingEstimate(b, i);
				//				System.out.println("Main released " + b + "," + i);
			}

			queue.add(0);
			assert queue.size() == markingsReached - closedActions;

			byte[] marking_m = new byte[numPlaces];

			while (!queue.isEmpty()) {

				assert queue.size() == markingsReached - closedActions;

				int m = queue.peek();
				int bm = m >>> blockBit;
				int im = m & blockMask;

				//				System.out.println("Main waiting for " + bm + "," + im);
				getLockForComputingEstimate(bm, im);
				//				System.out.println("Main locking " + bm + "," + im);
				try {
					if (m != queue.peek()) {
						// a parallel thread may have demoted m because the heuristic
						// changed from estimated to exact.
						continue;
					}

					if (debug == Debug.NORMAL && pollActions % 10000 == 0) {
						writeStatus();
					}
					m = queue.poll();
					pollActions++;

					if (isFinal(m)) {
						return handleFinalMarkingReached(startTime, m);
					}

					fillMarking(marking_m, bm, im);
					if (!hasExactHeuristic(bm, im)) {

						// compute the exact heuristic
						heuristic = getExactHeuristic(m, marking_m, bm, im);

						if (heuristic == HEURISTICINFINITE) {
							// marking from which final marking is unreachable
							// ignore state and continue

							// set the score to exact score
							setHScore(bm, im, heuristic, true);
							setClosed(im, im);
							closedActions++;
							continue;
						} else if (heuristic > getHScore(bm, im)) {
							// if the heuristic is higher push the head of the queue down
							// set the score to exact score
							setHScore(bm, im, heuristic, true);
							addToQueue(m);

							continue;
						} else {
							// continue with this marking
							// set the score to exact score
							setHScore(bm, im, heuristic, true);
						}
					}
					// add m to the closed set
					setClosed(bm, im);
					closedActions++;
				} finally {
					// release the lock after potentially closing the marking
					releaseLockForComputingEstimate(bm, im);
					//					System.out.println("Main released " + bm + "," + im);
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
						byte[] marking_n = fire(marking_m, t, bm, im);

						// check if n already reached before
						int newIndex = block * blockSize + indexInBlock;
						int n = visited.add(marking_n, newIndex);
						// adding the marking to the algorithm is handled by the VisitedSet.

						int bn = n >>> blockBit;
						int in = n & blockMask;
						getLockForComputingEstimate(bn, in);

						try {
							if (n == newIndex) {
								markingsReached++;
							}

							//					System.out.println("   Fire " + t + ": " + Utils.print(getMarking(n), net.numPlaces()));

							if (!isClosed(bn, in)) {

								// n is a fresh marking, not in the closed set
								// compute the F score on this path
								int tmpG = getGScore(bm, im) + net.getCost(t);

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
									addToQueue(n);

								} else if (!hasExactHeuristic(bn, in)) {
									//tmpG >= getGScore(n), i.e. we reached state n through a longer path.

									// G shore might not be an improvement, but see if we can derive the 
									// H score. 
									deriveOrEstimateHValue(m, bm, im, t, n, bn, in);

									if (hasExactHeuristic(bn, in)) {
										debug.writeEdgeTraversed(this, m, t, n, "color=blue");
										// marking is now exact and was not before. 
										assert queue.contains(n);
										addToQueue(n);
									}
								} else {
									debug.writeEdgeTraversed(this, m, t, n, "color=purple");
								}
							} else {
								debug.writeEdgeTraversed(this, m, t, n, "color=gray");
							}
						} finally {
							releaseLockForComputingEstimate(bn, in);
						} // end Try processing n
					} // end If enabled
				} // end for transitions
				processedMarking(m, bm, im);
			} // end While
			alignmentResult &= ~Utils.OPTIMALALIGNMENT;
			alignmentResult |= Utils.FAILEDALIGNMENT;
			runTime = (int) ((System.nanoTime() - startTime) / 1000);
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

	protected void addToQueue(int marking) {
		queue.add(marking);
		queueActions++;
	}

	protected void processedMarking(int marking, int blockMarking, int indexInBlock) {
		// skip. Can be used by subclasses to handle
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
	private transient final byte[] firingMarking;

	protected byte[] fire(byte[] fromMarking, short transition, int block, int index) {
		// fire transition t in marking stored at block, index
		// First consumption:
		short[] input = net.getInput(transition);
		short[] output = net.getOutput(transition);
		System.arraycopy(fromMarking, 0, firingMarking, 0, numPlaces);

		for (int i = input.length; i-- > 0;) {
			firingMarking[input[i]]--;
		}
		for (int i = output.length; i-- > 0;) {
			firingMarking[output[i]]++;
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

		return firingMarking;
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
	 * returns true if there is a place common in the output set of
	 * transitionFrom and the input set of transitionTo
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

	private byte[] fillMarking(byte[] marking, int block, int index) {
		// add initial marking
		System.arraycopy(net.getInitialMarking(), 0, marking, 0, numPlaces);
		// fire all transitions in the sequence back
		short t;
		int m = getPredecessor(block, index);
		while (m != NOPREDECESSOR) {
			t = getPredecessorTransition(block, index);
			block = m >>> blockBit;
			index = m & blockMask;
			unfire(marking, t);
			m = getPredecessor(block, index);
		}
		return marking;
	}

	public byte[] getMarking(int marking) {

		return fillMarking(new byte[numPlaces], marking >>> blockBit, marking & blockMask);
	}

	protected void fillMarking(byte[] markingArray, int marking) {
		fillMarking(markingArray, marking >>> blockBit, marking & blockMask);
	}

	public int addNewMarking(byte[] marking) {
		// allocate space for writing marking information in the block
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
		debug.writeDebugInfo(Debug.NORMAL, "Markings polled:   " + String.format("%,d", pollActions));
		debug.writeDebugInfo(Debug.NORMAL, "   Markings reached:" + String.format("%,d", markingsReached));
		debug.writeDebugInfo(Debug.NORMAL, "   Markings closed: " + String.format("%,d", closedActions));
		debug.writeDebugInfo(Debug.NORMAL, "   FScore head:     " + getFScore(queue.peek()) + " = G: "
				+ getGScore(queue.peek()) + " + H: " + getHScore(queue.peek()));
		debug.writeDebugInfo(Debug.NORMAL, "   Queue size:      " + String.format("%,d", queue.size()));
		debug.writeDebugInfo(Debug.NORMAL, "   Queue actions:   " + String.format("%,d", queueActions));
		debug.writeDebugInfo(Debug.NORMAL, "   Heuristics compu:" + String.format("%,d", heuristicsComputed));
		debug.writeDebugInfo(Debug.NORMAL, "   Heuristics deriv:" + String.format("%,d", heuristicsDerived));
		debug.writeDebugInfo(Debug.NORMAL, "   Heuristics est  :"
				+ String.format("%,d", (markingsReached - heuristicsComputed - heuristicsDerived)));
		debug.writeDebugInfo(Debug.NORMAL, "   Estimated memory:" + String.format("%,d", getEstimatedMemorySize()));
		double time = (System.nanoTime() - startConstructor) / 1000000.0;
		debug.writeDebugInfo(Debug.NORMAL, "   Time (ms):       " + String.format("%,f", time));
	}

	/**
	 * Grow the internal array structure. Method should be considered
	 * synchronized as it should not be executed in parallel.
	 */
	protected void growArrays() {
		if (block + 1 >= c_p.length) {
			int newLength = c_p.length < 64 ? c_p.length * 2 : (c_p.length * 3) / 2;
			if (newLength <= block + 1) {
				newLength = block + 2;
			}
			e_g_h_pt = Arrays.copyOf(e_g_h_pt, newLength);
			c_p = Arrays.copyOf(c_p, newLength);
		}
		// increase the block pointer
		block++;
		// reset the index in block
		indexInBlock = 0;

		// e_g_h_pt holds blocksize values for
		// estimated, g-score, h-score, predecessor transitions
		e_g_h_pt[block] = new long[blockSize];
		Arrays.fill(e_g_h_pt[block], GMASK);
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
		return (int) ((e_g_h_pt[block][index] & GMASK) >>> GSHIFT);
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
		e_g_h_pt[block][index] &= ~GMASK;
		long scoreL = ((long) score) << GSHIFT;
		if ((scoreL & GMASK) != scoreL) {
			alignmentResult |= Utils.COSTFUNCTIONOVERFLOW;
			e_g_h_pt[block][index] |= scoreL & GMASK;
		} else {
			e_g_h_pt[block][index] |= scoreL;
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
		return (int) ((e_g_h_pt[block][index] & HMASK) >>> HSHIFT);
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
		long scoreL = ((long) score) << HSHIFT;
		assert (scoreL & HMASK) == scoreL;
		// overwrite the last three bytes of the score.
		e_g_h_pt[block][index] &= ~HMASK; // reset to 0
		e_g_h_pt[block][index] |= scoreL; // set score
		if (isExact) {
			e_g_h_pt[block][index] |= EXACTMASK; // set exactFlag
		} else {
			e_g_h_pt[block][index] &= ~EXACTMASK; // clear exactFlag

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
		return (short) (e_g_h_pt[block][index] & PTMASK);
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
		e_g_h_pt[block][index] &= ~PTMASK; //clear pt bits
		e_g_h_pt[block][index] |= transition; //clear pt bits
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
		return (e_g_h_pt[block][index] & EXACTMASK) == EXACTMASK;
	}

	/**
	 * Signal that we intent to start computing an estimate for a specific
	 * marking
	 * 
	 * If the marking already has an exact estimate or is already closed, this
	 * method returns without changing anything.
	 * 
	 * If the marking is not exact and currently not computing, this method
	 * flags the marking to the computing state and returns
	 * 
	 * If the marking is not exact and the computing flag is set, this method
	 * blocks until the computing flag is released. After this, the marking may
	 * have an exact heuristic, but it may also be an estimated one.
	 * 
	 * A lock that is acquired should be released after setting the new exact
	 * heuristic value
	 * 
	 * @param b
	 * @param i
	 */
	protected void getLockForComputingEstimate(int b, int i) {
		synchronized (e_g_h_pt[b]) {
			while (isComputing(b, i)) {
				// currently computing, so lock cannot be obtained
				try {
					// wait until not computing anymore
					e_g_h_pt[b].wait(100);
				} catch (InterruptedException e) {
				}
			}
			// lock can be obtained
			// set the computing flag
			e_g_h_pt[b][i] |= COMPUTINGMASK;
		}
	}

	/**
	 * Signal that we completed computing an exact estimate for a specific
	 * marking.
	 * 
	 * @param b
	 * @param i
	 */
	protected void releaseLockForComputingEstimate(int b, int i) {
		synchronized (e_g_h_pt[b]) {
			e_g_h_pt[b][i] &= ~COMPUTINGMASK;
			e_g_h_pt[b].notify();
		}
	}

	protected boolean isComputing(int block, int index) {
		return (e_g_h_pt[block][index] & COMPUTINGMASK) == COMPUTINGMASK;
	}

	private transient final byte[] equalMarking;

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
		fillMarking(equalMarking, marking1);
		return Arrays.equals(equalMarking, marking2);
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

	private transient final byte[] hashCodeMarking;

	/**
	 * Returns the hashCode of a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int hashCode(int marking) {
		fillMarking(hashCodeMarking, marking);
		return hashCode(hashCodeMarking);
		//		int b = marking >>> blockBit;
		//		int i = marking & blockMask;
		//		return hashCodeInternal(markingLo[b], bm * i, bm, markingHi[b], bm * i, bm);
	}

	/**
	 * Returns the hashCode of a stored marking which is provided as an array of
	 * length 2*bm, where the first bm bytes provide the low bits and the second
	 * bm bytes provide the high bits.
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

	public boolean isComputing(int i) {
		return isComputing(i >>> blockBit, i & blockMask);
	}

	public SyncProduct getNet() {
		return net;
	}

}
