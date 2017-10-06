package nl.tue.alignment.algorithms;

import gnu.trove.iterator.TShortIterator;
import gnu.trove.list.TShortList;
import gnu.trove.list.array.TShortArrayList;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.TShortObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TShortObjectHashMap;

import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.ReplayAlgorithm;
import nl.tue.alignment.SyncProduct;
import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.astar.util.ilp.LPMatrixException;

/**
 * Implements a variant of AStar shortest path algorithm for alignments. It uses
 * (I)LP to estimate the remaining distance.
 * 
 * This implementation can NOT be used for prefix alignments. The final marking
 * has to be reached as this is assumed in the underlying (I)LP.
 * 
 * @author bfvdonge
 * 
 */
public class AStarLargeLP extends ReplayAlgorithm {

	private final Object estimatedLock = new Object();

	//	private final class BlockMonitor implements Runnable {
	//
	//		private final int currentBlock;
	//
	//		private final double[] varsBlockMonitor;
	//
	//		public BlockMonitor(int currentBlock, int numTrans) {
	//			this.currentBlock = currentBlock;
	//			this.varsBlockMonitor = new double[numTrans];
	//			synchronized (AStarLargeLP.this) {
	//				lpSolutionsSize += 12 + 4 + 8 + 12 + 4 + numTrans * 8;
	//			}
	//		}
	//
	//		public void run() {
	//			int b = currentBlock;
	//			int i = 0;
	//			do {
	//				int m = b * blockSize + i;
	//				if (m < markingsReached) {
	//					//					System.out.println("Monitor waiting for " + b + "," + i);
	//					getLockForComputingEstimate(b, i);
	//					//					System.out.println("Monitor locking " + b + "," + i);
	//					try {
	//						if (!isClosed(b, i) && !hasExactHeuristic(b, i)) {
	//							// an open marking without an exact solution for the heuristic
	//							LpSolve solver = provider.firstAvailable();
	//							int heuristic;
	//							try {
	//								heuristic = getExactHeuristic(solver, m, getMarking(m), b, i, varsBlockMonitor);
	//							} finally {
	//								provider.finished(solver);
	//							}
	//							if (heuristic > getHScore(b, i)) {
	//								setHScore(b, i, heuristic, true);
	//								// sort the marking in the queue
	//								assert queue.contains(m);
	//								queue.add(m);
	//								queueActions++;
	//							} else {
	//								setHScore(b, i, heuristic, true);
	//							}
	//						}
	//					} finally {
	//						//						System.out.println("Monitor releasing " + b + "," + (m & blockMask));
	//						releaseLockForComputingEstimate(b, m & blockMask);
	//					}
	//					i++;
	//				} else {
	//					synchronized (e_g_h_pt[b]) {
	//						try {
	//							e_g_h_pt[b].wait(200);
	//						} catch (InterruptedException e) {
	//						}
	//					}
	//				}
	//			} while (!threadpool.isShutdown() && i < blockSize);
	//			synchronized (AStarLargeLP.this) {
	//				lpSolutionsSize -= 12 + 4 + 8 + 12 + 4 + varsBlockMonitor.length * 8;
	//			}
	//
	//			//			System.out.println("Closing monitor for block " + b + ". Threadpool shutdown: " + threadpool.isShutdown()
	//			//					+ ".");
	//
	//		}
	//	}

	// for each stored solution, the first byte is used for flagging.
	// the first bit indicates whether the solution is derived
	// The next three bits store the number of bits per transition (0 implies 1 bit per transition, 7 implies 8 bits per transition)
	// The rest of the array  then stores the solution
	protected static final byte COMPUTED = (byte) 0b00000000;
	protected static final byte DERIVED = (byte) 0b10000000;

	protected static final byte BITPERTRANSMASK = (byte) 0b01110000;
	protected static final byte FREEBITSFIRSTBYTE = 4;

	// stores the location of the LP solution plus a flag if it is derived or real
	protected TIntObjectMap<byte[]> lpSolutions = new TIntObjectHashMap<>(16);
	protected long lpSolutionsSize = 4;

	protected int bytesUsed;
	protected long solveTime = 0;

	//	protected int numRows;
	//	protected int numCols;
	private short[] indexMap;

	private final TShortObjectMap<TShortList> trans2LSMove = new TShortObjectHashMap<>();

	private ExecutorService threadpool;

	private final boolean doMultiThreading;

	private LpSolve solver;

	private int numEvents;

	private int modelMoves;

	private SyncProduct product;

	public AStarLargeLP(SyncProduct product) throws LPMatrixException {
		this(product, true, true, true, false, false, Debug.NONE);
	}

	public AStarLargeLP(SyncProduct product, boolean moveSorting, boolean queueSorting, boolean preferExact,
			boolean isInteger, boolean doMultiThreading, Debug debug) throws LPMatrixException {
		super(product, moveSorting, queueSorting, preferExact, debug);
		this.product = product;
		this.doMultiThreading = doMultiThreading;

		trans2LSMove.put(SyncProduct.NOEVENT, new TShortArrayList(10));
		numEvents = -1;
		TShortList set;
		for (short t = 0; t < product.numTransitions(); t++) {
			short e = product.getEventOf(t);
			set = trans2LSMove.get(e);
			if (set == null) {
				set = new TShortArrayList(3);
				trans2LSMove.put(e, set);
			}
			set.add(t);
			if (product.getEventOf(t) > numEvents) {
				numEvents = product.getEventOf(t);
			}
		}
		modelMoves = trans2LSMove.get(SyncProduct.NOEVENT).size();
		numEvents++;

		indexMap = new short[(numEvents - 1) * modelMoves + product.numTransitions()];

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);
	}

	private void init() throws LPMatrixException {
		int monitorThreads;

		try {
			solver = LpSolve.makeLp(numEvents * product.numPlaces(), 0);
			solver.setAddRowmode(false);

			double[] col = new double[1 + numEvents * product.numPlaces()];
			short[] input, output;
			int c = 0;

			int start = 1;
			for (short e = 0; e < numEvents; e++) {
				if (trans2LSMove.get(e) != null) {
					TShortIterator it = trans2LSMove.get(SyncProduct.NOEVENT).iterator();
					// first the model moves in this block
					while (it.hasNext()) {
						Arrays.fill(col, 0);
						short t = it.next();
						input = product.getInput(t);
						for (int i = 0; i < input.length; i++) {
							for (int p = start + input[i]; p < col.length; p += product.numPlaces()) {
								col[p] -= 1;
							}
						}
						output = product.getOutput(t);
						for (int i = 0; i < output.length; i++) {
							for (int p = start + output[i]; p < col.length; p += product.numPlaces()) {
								col[p] += 1;
							}
						}
						solver.addColumn(col);
						indexMap[c] = t;
						c++;
						solver.setUpbo(c, 0);
						solver.setUpbo(c, 255);
						solver.setInt(c, false);
						solver.setObj(c, product.getCost(t));
					}
					it = trans2LSMove.get(e).iterator();
					while (it.hasNext()) {
						Arrays.fill(col, 0);
						short t = it.next();
						input = product.getInput(t);
						for (int i = 0; i < input.length; i++) {
							for (int p = start + input[i]; p < col.length; p += product.numPlaces()) {
								col[p] -= 1;
							}
						}
						output = product.getOutput(t);
						for (int i = 0; i < output.length; i++) {
							for (int p = start + output[i]; p < col.length; p += product.numPlaces()) {
								col[p] += 1;
							}
						}
						solver.addColumn(col);
						indexMap[c] = t;
						c++;
						solver.setUpbo(c, 0);
						solver.setUpbo(c, 1);
						solver.setInt(c, false);
						solver.setObj(c, product.getCost(t));
					}
				}
				start += product.numPlaces();
			}

			int r;
			for (r = 1; r <= (numEvents - 1) * product.numPlaces(); r++) {
				solver.setConstrType(r, LpSolve.GE);
				solver.setRh(r, -product.getInitialMarking()[(r - 1) % product.numPlaces()]);
			}
			for (; r <= numEvents * product.numPlaces(); r++) {
				solver.setConstrType(r, LpSolve.EQ);
				solver.setRh(r, product.getFinalMarking()[(r - 1) % product.numPlaces()]
						- product.getInitialMarking()[(r - 1) % product.numPlaces()]);
			}
			solver.setMinim();
			solver.setVerbose(0);

			//			int res = solver.solve();
			//			double[] vars = new double[indexMap.length];
			//			solver.getVariables(vars);
			//			System.out.println(res + " : " + Arrays.toString(vars));
		} catch (LpSolveException e) {
			throw new LPMatrixException(e);
		}

		if (doMultiThreading) {
			monitorThreads = Math.max(1, Runtime.getRuntime().availableProcessors() / 2);
			threadpool = Executors.newFixedThreadPool(monitorThreads);

		} else {
			monitorThreads = 0;
			threadpool = null;
		}
		varsMainThread = new double[indexMap.length];
		tempForSettingSolution = new int[net.numTransitions()];

	}

	@Override
	protected void growArrays() {
		super.growArrays();
		if (doMultiThreading) {
			//			threadpool.submit(new BlockMonitor(block, net.numTransitions()));
		}
	}

	@Override
	public short[] run() throws LPMatrixException {
		long start = System.nanoTime();
		init();
		return super.runReplayAlgorithm(start);
	}

	protected double[] varsMainThread;

	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		// find an available solver and block until one is available.

		return getExactHeuristic(solver, marking, markingArray, markingBlock, markingIndex, varsMainThread);
	}

	protected int getLastEventOf(int marking) {
		int m = marking;
		short trans = getPredecessorTransition(m);
		while (net.getEventOf(trans) < 0 && m > 0) {
			m = getPredecessor(m);
			trans = getPredecessorTransition(m);
		}
		int evt = net.getEventOf(trans);
		return evt;
	}

	private int getExactHeuristic(LpSolve solver, int marking, byte[] markingArray, int markingBlock, int markingIndex,
			double[] vars) {

		long start = System.nanoTime();
		// start from correct right hand side
		try {

			int e = getLastEventOf(marking);

			int r;
			for (r = 1; r <= (e + 1) * product.numPlaces(); r++) {
				solver.setRh(r, 0);
			}
			for (; r <= (numEvents - 1) * product.numPlaces(); r++) {
				solver.setRh(r, -markingArray[(r - 1) % product.numPlaces()]);
			}
			for (; r <= numEvents * product.numPlaces(); r++) {
				solver.setRh(r, product.getFinalMarking()[(r - 1) % product.numPlaces()]
						- markingArray[(r - 1) % product.numPlaces()]);
			}

			solver.defaultBasis();
			int solverResult = solver.solve();
			synchronized (this) {
				heuristicsComputed++;
			}

			//			if (solverResult == LpSolve.INFEASIBLE || solverResult == LpSolve.NUMFAILURE) {
			//				// BVD: LpSolve has the tendency to give false infeasible or numfailure answers. 
			//				// It's unclear when or why this happens, but just in case...
			//				solverResult = solver.solve();
			//
			//			}
			if (solverResult == LpSolve.OPTIMAL) {
				// retrieve the solution
				solver.getVariables(vars);
				setNewLpSolution(marking, vars);

				// compute cost estimate
				double c = computeCostForVars(vars);

				if (c >= HEURISTICINFINITE) {
					synchronized (this) {
						alignmentResult |= Utils.HEURISTICFUNCTIONOVERFLOW;
					}
					// continue with maximum heuristic value not equal to infinity.
					return HEURISTICINFINITE - 1;
				}

				// assume precision 1E-9 and round down
				return (int) (c + 1E-9);
			} else if (solverResult == LpSolve.INFEASIBLE) {

				return HEURISTICINFINITE;
			} else {
				//					lp.writeLp("D:/temp/alignment/debugLP-Alignment.lp");
				//System.err.println("Error code from LpSolve solver:" + solverResult);
				return HEURISTICINFINITE;
			}

		} catch (LpSolveException e) {
			return HEURISTICINFINITE;
		} finally {
			synchronized (this) {
				solveTime += System.nanoTime() - start;
			}
		}

	}

	protected double computeCostForVars(double[] vars) {
		double c = 0;
		for (int t = vars.length; t-- > 0;) {
			c += vars[t] * net.getCost(indexMap[t]);
		}
		return c;
	}

	protected synchronized int getLpSolution(int marking, short transition) {
		byte[] solution = getSolution(marking);
		//		if ((solution[0] & STOREDFULL) == STOREDFULL) {
		// get the bits used per transition
		int bits = 1 + ((solution[0] & BITPERTRANSMASK) >>> FREEBITSFIRSTBYTE);
		// which is the first bit?
		int fromBit = 8 - FREEBITSFIRSTBYTE + transition * bits;
		// that implies the following byte
		int fromByte = fromBit >>> 3;
		// with the following index in byte.
		fromBit &= 7;

		byte currentBit = (byte) (1 << (7 - fromBit));
		int value = 0;
		for (int i = 0; i < bits; i++) {
			// shift value left
			value <<= 1;

			// flip the bit
			if ((solution[fromByte] & currentBit) != 0)
				value++;

			// rotate bit right 
			currentBit = (byte) (((currentBit & 0xFF) >>> 1) | (currentBit << 7));
			// increase byte if needed.
			if (currentBit < 0)
				fromByte++;

		}

		return value;
	}

	protected synchronized boolean isDerivedLpSolution(int marking) {
		return getSolution(marking) != null && (getSolution(marking)[0] & DERIVED) == DERIVED;
	}

	protected synchronized void setDerivedLpSolution(int from, int to, short transition) {
		assert getSolution(to) == null;
		byte[] solutionFrom = getSolution(from);

		byte[] solution = Arrays.copyOf(solutionFrom, solutionFrom.length);

		solution[0] |= DERIVED;

		// get the length of the bits used per transition
		int bits = 1 + ((solution[0] & BITPERTRANSMASK) >>> FREEBITSFIRSTBYTE);
		// which is the least significant bit?
		int fromBit = 8 - FREEBITSFIRSTBYTE + transition * bits + (bits - 1);
		// that implies the following byte
		int fromByte = fromBit >>> 3;
		// with the following index in byte.
		fromBit &= 7;
		// most significant bit in fromBit
		byte lsBit = (byte) (1 << (7 - fromBit));

		// we need to reduce by 1.
		for (int i = 0; i < bits; i++) {
			// flip the bit
			if ((solution[fromByte] & lsBit) != 0) {
				// first bit that is 1. Flip and terminate
				solution[fromByte] ^= lsBit;
				//					assert getLpSolution(to, transition) == getLpSolution(from, transition) - 1;
				addSolution(to, solution);
				return;
			}
			// flip and continue;
			solution[fromByte] ^= lsBit;
			// rotate bit left
			lsBit = (byte) (((lsBit & 0xFF) >>> 7) | (lsBit << 1));
			// decrease byte if needed.
			if (lsBit == 1)
				fromByte--;

		}
		assert false;

		//		}
	}

	private int[] tempForSettingSolution;

	protected synchronized void setNewLpSolution(int marking, double[] solutionDouble) {
		// copy the solution from double array to byte array (rounding down)
		// and compute the maximum.
		Arrays.fill(tempForSettingSolution, 0);
		byte bits = 1;
		for (int i = solutionDouble.length; i-- > 0;) {
			tempForSettingSolution[indexMap[i]] += ((int) (solutionDouble[i] + 5E-11));
			if (tempForSettingSolution[indexMap[i]] > (1 << bits)) {
				bits++;
			}
		}

		// to store this solution, we need "bits" bits per transition
		// plus a header consisting of 8-FREEBITSFIRSTBYTE bits.
		// this translate to 
		int bytes = 8 - FREEBITSFIRSTBYTE + (tempForSettingSolution.length * bits + 4) / 8;

		assert getSolution(marking) == null;
		byte[] solution = new byte[bytes];

		// set the computed flag in the first two bits
		solution[0] = COMPUTED;
		// set the number of bits used in the following 3 bits
		bits--;
		solution[0] |= bits << FREEBITSFIRSTBYTE;

		int currentByte = 0;
		byte currentBit = (1 << (FREEBITSFIRSTBYTE - 1));
		for (short t = 0; t < tempForSettingSolution.length; t++) {
			// tempForSettingSolution[i] can be stored in "bits" bits.
			for (int b = 1 << bits; b > 0; b >>>= 1) {
				// copy the appropriate bit
				if ((tempForSettingSolution[t] & b) != 0)
					solution[currentByte] |= currentBit;

				// rotate right
				currentBit = (byte) ((((currentBit & 0xFF) >>> 1) | (currentBit << 7)));
				if (currentBit < 0)
					currentByte++;

			}
			//			assert getLpSolution(marking, t) == tempForSettingSolution[t];
		}
		addSolution(marking, solution);
	}

	private byte[] getSolution(int marking) {
		return lpSolutions.get(marking);
	}

	private void addSolution(int marking, byte[] solution) {
		lpSolutions.put(marking, solution);
		lpSolutionsSize += 12 + 4 + solution.length; // object size
		lpSolutionsSize += 1 + 4 + 8; // used flag + key + value pointer
	}

	/**
	 * In ILP version, only one given final marking is the target.
	 */
	protected boolean isFinal(int marking) {
		return equalMarking(marking, net.getFinalMarking());
	}

	protected void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to,
			int toBlock, int toIndex) {
		if (hasExactHeuristic(fromBlock, fromIndex) && (getLpSolution(from, transition) >= 1)) {
			// from Marking has exact heuristic
			// we can derive an exact heuristic from it

			setDerivedLpSolution(from, to, transition);
			// set the exact h score
			setHScore(toBlock, toIndex, getHScore(fromBlock, fromIndex) - net.getCost(transition), true);
			heuristicsDerived++;

		} else {
			if (isFinal(to)) {
				setHScore(toBlock, toIndex, 0, true);
			}
			int h = getHScore(fromBlock, fromIndex) - net.getCost(transition);
			if (h < 0) {
				h = 0;
			}
			if (h > getHScore(toBlock, toIndex)) {
				// estimated heuristic should not decrease.
				setHScore(toBlock, toIndex, h, false);
				heuristicsEstimated++;
			}
		}

	}

	@Override
	protected void addToQueue(int marking) {
		super.addToQueue(marking);
		if (!hasExactHeuristic(marking)) {
			synchronized (estimatedLock) {
				estimatedLock.notifyAll();
			}
		}
	}

	@Override
	protected long getEstimatedMemorySize() {
		long val = super.getEstimatedMemorySize();
		// count space for all computed solutions
		val += lpSolutionsSize;
		// count size of matrix
		val += bytesUsed;
		// count size of rhs
		return val;
	}

	@Override
	public TObjectIntMap<Utils.Statistic> getStatistics() {
		TObjectIntMap<Statistic> map = super.getStatistics();
		map.put(Statistic.HEURISTICTIME, (int) (solveTime / 1000));
		return map;
	}

	@Override
	protected void writeEndOfAlignmentDot() {
		TObjectIntMap<Statistic> map = getStatistics();
		for (int m = 0; m < markingsReached; m++) {
			if (isDerivedLpSolution(m)) {
				debug.writeMarkingReached(this, m, "color=blue");
			} else {
				debug.writeMarkingReached(this, m);
			}
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

	@Override
	protected void processedMarking(int marking, int blockMarking, int indexInBlock) {
		super.processedMarking(marking, blockMarking, indexInBlock);
		synchronized (this) {
			lpSolutionsSize -= 12 + 4 + lpSolutions.remove(marking).length; // object size
			lpSolutionsSize -= 1 + 4 + 8; // used flag + key + value pointer
		}
	}

	protected void terminateRun() {
		try {
			super.terminateRun();
		} finally {
			if (doMultiThreading && !threadpool.isShutdown()) {
				threadpool.shutdown();
				//				System.out.println("Threadpool shutdown upon termination of run.");
				do {
					synchronized (estimatedLock) {
						estimatedLock.notifyAll();
					}
					try {
						threadpool.awaitTermination(100, TimeUnit.MILLISECONDS);
					} catch (InterruptedException e) {
					}
				} while (!threadpool.isTerminated());
			}
			solver.deleteAndRemoveLp();
		}
	}

}