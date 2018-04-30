package nl.tue.alignment.algorithms;

import java.util.Arrays;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.syncproduct.SyncProduct;
import nl.tue.astar.util.ilp.LPMatrix;
import nl.tue.astar.util.ilp.LPMatrix.SPARSE.LPSOLVE;
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
public class AStar extends ReplayAlgorithm {

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
	//			synchronized (AStar.this) {
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
	//							synchronized (queue) {
	//								if (heuristic > getHScore(b, i)) {
	//									assert queue.checkInv();
	//									setHScore(b, i, heuristic, true);
	//									// sort the marking in the queue
	//									assert queue.contains(m);
	//									queue.add(m);
	//									queueActions++;
	//									assert queue.checkInv();
	//								} else {
	//									assert heuristic == getHScore(b, i);
	//									setHScore(b, i, heuristic, true);
	//								}
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
	//			synchronized (AStar.this) {
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

	protected final double[] rhf;
	protected int bytesUsed;
	protected long solveTime = 0;

	//	protected int numRows;
	//	protected int numCols;

	//	private ExecutorService threadpool;
	//	private LPProblemProvider provider;
	private LpSolve solver;

	//	private final int numberOfThreads;

	private LPSOLVE matrix;

	public AStar(SyncProduct product) throws LPMatrixException {
		this(product, true, true, true, false, Debug.NONE);
	}

	public AStar(SyncProduct product, boolean moveSorting, boolean queueSorting, boolean preferExact, boolean isInteger,
			Debug debug) throws LPMatrixException {
		super(product, moveSorting, queueSorting, preferExact, debug);
		//		this.numberOfThreads = numberOfThreads;
		//		this.numRows = net.numPlaces();
		matrix = new LPMatrix.SPARSE.LPSOLVE(net.numPlaces(), net.numTransitions());

		// Set the objective to follow the cost function
		for (short t = net.numTransitions(); t-- > 0;) {
			matrix.setObjective(t, net.getCost(t));

			short[] input = net.getInput(t);
			for (int i = input.length; i-- > 0;) {
				// transition t consumes from place p, hence  incidence matrix
				// is -1;
				matrix.adjustMat(input[i], t, -1);
				assert matrix.getMat(input[i], t) <= 1;
			}
			short[] output = net.getOutput(t);
			for (int i = output.length; i-- > 0;) {
				matrix.adjustMat(output[i], t, 1);
				assert matrix.getMat(output[i], t) <= 1;
			}

			// Use integer variables if specified
			matrix.setInt(t, isInteger);
			// Set lower bound of 0
			matrix.setLowbo(t, 0);
			// Set upper bound to 255 (we assume unsigned byte is sufficient to store result)
			matrix.setUpbo(t, 255);

			if (debug != Debug.NONE) {
				matrix.setColName(t, net.getTransitionLabel(t));
			}

		}

		rhf = new double[net.numPlaces()];
		byte[] marking = net.getFinalMarking();
		for (short p = net.numPlaces(); p-- > 0;) {
			if (debug != Debug.NONE) {
				matrix.setRowName(p, net.getPlaceLabel(p));
			}
			// set the constraint to equality
			matrix.setConstrType(p, LPMatrix.EQ);
			rhf[p] = marking[p];
		}
		matrix.setMinim();

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);
	}

	@Override
	protected void initializeIteration() throws LPMatrixException {
		//		try {
		solver = matrix.toSolver();
		//			if (numberOfThreads > 1) {
		//				provider = new LPProblemProvider(solver.copyLp(), numberOfThreads);
		//				threadpool = Executors.newFixedThreadPool(numberOfThreads);
		//
		//			} else {
		//				provider = null;
		//				threadpool = null;
		//			}
		//		} catch (LpSolveException e) {
		//			throw new LPMatrixException(e);
		//		}
		// bytes for solver
		bytesUsed = matrix.bytesUsed();
		// bytes for solvers inside monitorthreads
		//bytesUsed += matrix.bytesUsed();//* numberOfThreads;
		//		numCols = net.numTransitions() + 1;
		varsMainThread = new double[net.numTransitions()];
		tempForSettingSolution = new int[net.numTransitions()];
		super.initializeIterationInternal();
	}

	@Override
	protected void growArrays() {
		super.growArrays();
		//		if (numberOfThreads > 1) {
		//			threadpool.submit(new BlockMonitor(block, net.numTransitions()));
		//		}
	}

	protected double[] varsMainThread;

	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		// find an available solver and block until one is available.

		return getExactHeuristic(solver, marking, markingArray, markingBlock, markingIndex, varsMainThread);
	}

	private int getExactHeuristic(LpSolve solver, int marking, byte[] markingArray, int markingBlock, int markingIndex,
			double[] vars) {

		long start = System.nanoTime();
		// start from correct right hand side
		try {
			for (int p = net.numPlaces(); p-- > 0;) {
				// set right hand side to final marking 
				solver.setRh(p + 1, rhf[p] - markingArray[p]);
			}

			solver.defaultBasis();
			solver.setTimeout(Math.max(1, timeoutAtTimeInMillisecond - System.currentTimeMillis()) / 1000);
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
		for (short t = net.numTransitions(); t-- > 0;) {
			c += vars[t] * net.getCost(t);
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
		byte bits = 1;
		for (int i = solutionDouble.length; i-- > 0;) {
			tempForSettingSolution[i] = ((int) (solutionDouble[i] + 5E-11));
			if (tempForSettingSolution[i] > (1 << bits)) {
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

	protected void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to, int toBlock,
			int toIndex) {
		if (hasExactHeuristic(fromBlock, fromIndex) && getHScore(fromBlock, fromIndex) != HEURISTICINFINITE
				&& (getLpSolution(from, transition) >= 1)) {
			// from Marking has exact heuristic
			// we can derive an exact heuristic from it

			setDerivedLpSolution(from, to, transition);
			// set the exact h score
			setHScore(toBlock, toIndex, getHScore(fromBlock, fromIndex) - net.getCost(transition), true);
			heuristicsDerived++;

		} else if (hasExactHeuristic(fromBlock, fromIndex) && getHScore(fromBlock, fromIndex) == HEURISTICINFINITE) {
			// marking from which final state cannot be reached
			assert false;
			setHScore(toBlock, toIndex, HEURISTICINFINITE, true);
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
		val += rhf.length * 8 + 4;
		return val;
	}

	@Override
	protected void fillStatistics(short[] alignment) {
		super.fillStatistics(alignment);
		putStatistic(Statistic.HEURISTICTIME, (int) (solveTime / 1000));
	}

	@Override
	protected void writeEndOfAlignmentDot(short[] alignment, int markingsReachedInRun, int closedActionsInRun) {
		for (int m = 0; m < markingsReached; m++) {
			if (!isClosed(m)) {
				if (isDerivedLpSolution(m)) {
					debug.writeMarkingReached(this, m, "color=blue,style=bold");
				} else if (hasExactHeuristic(m)) {
					debug.writeMarkingReached(this, m, "style=bold");
				} else {
					debug.writeMarkingReached(this, m, "style=dashed");
				}
			}
		}
		StringBuilder b = new StringBuilder();
		b.append("info [shape=plaintext,label=<");
		for (Statistic s : Statistic.values()) {
			b.append(s);
			b.append(": ");
			b.append(replayStatistics.get(s));
			b.append("<br/>");
		}
		b.append(">];");

		debug.println(Debug.DOT, b.toString());
		debug.println(Debug.DOT, "}");
		debug.println(Debug.DOT, "}");
	}

	@Override
	protected void processedMarking(int marking, int blockMarking, int indexInBlock) {
		super.processedMarking(marking, blockMarking, indexInBlock);
		synchronized (this) {
			if (isDerivedLpSolution(marking)) {
				debug.writeMarkingReached(this, marking, "color=blue,style=bold");
			} else {
				debug.writeMarkingReached(this, marking, "style=bold");
			}
			lpSolutionsSize -= 12 + 4 + lpSolutions.remove(marking).length; // object size
			lpSolutionsSize -= 1 + 4 + 8; // used flag + key + value pointer
		}
	}

	protected void terminateIteration(short[] alignment, int markingsReachedInRun, int closedActionsInRun) {
		try {
			super.terminateIteration(alignment, markingsReachedInRun, closedActionsInRun);
		} finally {
			//			if (numberOfThreads > 1 && !threadpool.isShutdown()) {
			//				threadpool.shutdown();
			//				//				System.out.println("Threadpool shutdown upon termination of run.");
			//				do {
			//					synchronized (estimatedLock) {
			//						estimatedLock.notifyAll();
			//					}
			//					try {
			//						threadpool.awaitTermination(100, TimeUnit.MILLISECONDS);
			//					} catch (InterruptedException e) {
			//					}
			//				} while (!threadpool.isTerminated());
			//				provider.deleteLps();
			//			}
			solver.deleteAndRemoveLp();
		}
	}

}