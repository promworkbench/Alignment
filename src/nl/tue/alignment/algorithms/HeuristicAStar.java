package nl.tue.alignment.algorithms;

import gnu.trove.map.TObjectIntMap;

import java.util.Arrays;
import java.util.Random;

import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.ReplayAlgorithm;
import nl.tue.alignment.SyncProduct;
import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.astar.util.ilp.LPMatrix;
import nl.tue.astar.util.ilp.LPMatrixException;

public class HeuristicAStar extends ReplayAlgorithm {

	protected static int DEFAULTARRAYSIZE = 16;

	// stores the location of the LP solution plus a flag if it is derived or real
	protected byte[][] lpSolutions = new byte[0][];
	protected long lpSolutionsSize = 4;

	protected final double[] rhf;
	protected final int bytesUsed;
	protected final LpSolve solver;
	protected long solveTime = 0;
	protected int numRows;
	protected int numColumns;
	protected int blockedSolutions = 0;

	private final int targetFunctionRow;
	private final int randomRowNumber;
	private final double[] randomRow;

	public HeuristicAStar(SyncProduct product, Debug debug) throws LPMatrixException {
		super(product, true, true, false, debug);
		LPMatrix.SPARSE.LPSOLVE matrix = new LPMatrix.SPARSE.LPSOLVE(net.numPlaces() + 1, net.numTransitions());
		numColumns = net.numTransitions();

		// Set the objective to follow the cost function
		for (short t = net.numTransitions(); t-- > 0;) {
			matrix.setObjective(t, net.getCost(t));

			byte[] input = net.getInput(t);
			byte[] output = net.getOutput(t);
			for (int p = net.numPlaces(); p-- > 0;) {

				if ((input[p >>> 3] & (Utils.BYTEHIGHBIT >>> (p & 7))) != 0) {
					// transition t consumes from place p, hence  incidence matrix
					// is -1;
					matrix.adjustMat(p, t, -1);
				}
				if ((output[p >>> 3] & (Utils.BYTEHIGHBIT >>> (p & 7))) != 0) {
					// transition t produced in place p, hence  incidence matrix
					// is +1;
					matrix.adjustMat(p, t, 1);
				}
			}
			// Use integer variables if specified
			matrix.setInt(t, true);
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
			if ((marking[p >>> 3] & (Utils.BYTEHIGHBIT >>> (p & 7))) != 0) {
				// bit in low bits of the final marking;
				rhf[p] += 1;
			}
			if ((marking[(marking.length / 2) + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) != 0) {
				// bit in high bits of the final marking;
				rhf[p] += 2;
			}
		}

		targetFunctionRow = net.numPlaces();
		randomRowNumber = net.numPlaces();
		randomRow = new double[net.numTransitions() + 1];
		Random r = new Random();
		for (short t = 0; t < net.numTransitions(); t++) {
			matrix.setMat(targetFunctionRow, t, net.getCost(t));
			do {
				randomRow[t + 1] = net.numTransitions() * (1 - 2 * r.nextDouble());
			} while (randomRow[t + 1] > -1 && randomRow[t + 1] < 1);
		}
		matrix.setConstrType(targetFunctionRow, LPMatrix.LE);

		matrix.setMinim();
		solver = matrix.toSolver();
		bytesUsed = matrix.bytesUsed();
		numRows = randomRowNumber;
		vars = new double[numColumns];

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);

	}

	@Override
	public short[] run() {
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
			flushActions = 0;
			maxEstimated = 0;
			maxEstimatedLength = DEFAULTESTIMATEDSIZE;

			final long start = System.nanoTime();
			growArray(DEFAULTARRAYSIZE);
			lpSolutions[0] = new byte[net.numTransitions()];

			short[] result;
			do {
				result = runInternal(start);
			} while (result == null);
			return result;

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

	private short[] runInternal(long startTime) {
		visited.clear();

		markingLo = new byte[0][];
		markingHi = new byte[0][];
		ptl_g = new int[0][];
		e_pth_h = new int[0][];
		c_p = new int[0][];
		block = -1;
		indexInBlock = 0;
		growArrays();
		markingsReached = 0;

		// get the initial marking
		byte[] initialMarking = net.getInitialMarking();
		// add to the set of markings
		int pos = addNewMarking(initialMarking);
		int b = pos >>> blockBit;
		int i = pos & blockMask;
		// set predecessor to null
		setPredecessor(b, i, 0);
		setPredecessorTransition(b, i, (short) 0);
		setGScore(b, i, 0);

		byte[] lastSolution = lpSolutions[0];

		int heuristic = getExactHeuristic(0, b, i);
		setHScore(b, i, heuristic, true);

		queue.add(0);

		System.out.println("Starting with H=" + heuristic + " : ");

		for (short t = 0; t < net.numTransitions(); t++) {
			if (lastSolution[t] > lpSolutions[0][t]) {
				System.out.print(" -[" + net.getTransitionLabel(t) + "]");
			} else if (lastSolution[t] < lpSolutions[0][t]) {
				System.out.print(" +[" + net.getTransitionLabel(t) + "]");
			}
		}
		System.out.println();

		this.currentFScore = heuristic;

		while (!queue.isEmpty()) {

			// get the most promising marking m
			int m;
			m = queue.poll();
			pollActions++;

			int bm = m >>> blockBit;
			int im = m & blockMask;

			if (isFinal(m)) {
				return handleFinalMarkingReached(startTime, m);
			}
			debug.writeMarkingReached(this, m);
			// add m to the closed set
			setClosed(bm, im);
			closedActions++;

			if (debug == Debug.NORMAL && closedActions % 1000000 == 0) {
				writeStatus();
			}

			// iterate over all transitions
			for (short t = 0; t < net.numTransitions(); t++) {
				//				for (short t = net.numTransitions(); t-- > 0;) {
				//				for (short t : trans) {

				// check for enabling
				if (isEnabled(t, bm, im)) {
					edgesTraversed++;

					// t is allowed to fire.
					byte[] n_array = fire(t, bm, im);

					// check if n already reached before
					int n = visited.add(n_array, block * blockSize + indexInBlock);
					// adding the marking to the algorithm is handled by the VisitedSet.

					//					System.out.println("   Fire " + t + ": " + Utils.print(getMarking(n), net.numPlaces()));

					int bn = n >>> blockBit;
					int in = n & blockMask;
					if (!isClosed(bn, in)) {

						// n is a fresh marking, not in the closed set
						// compute the F score on this path
						int tmpG = getGScore(bm, im) + net.getCost(t);
						// update path if GScore equal and transtion lower order
						//						if (tmpG == getGScore(bn, in) && getPredecessorTransition(bn, in) < t) {
						//							// when sorting moves, enabled transitions are bound to the highest incoming
						//							// transition. For equal G score, the highest transition going into a marking is
						//							// therefore preferred.
						//							debug.writeEdgeTraversed(this, m, t, n, "color=green");
						//							setPredecessor(bn, in, m);
						//							setPredecessorTransition(bn, in, t);
						//
						//							add = false;
						//						}

						if (tmpG < getGScore(bn, in)) {
							debug.writeEdgeTraversed(this, m, t, n);

							// found a shorter path to n.
							setGScore(bn, in, tmpG);

							// set predecessor
							setPredecessor(bn, in, m);
							setPredecessorTransition(bn, in, t);

							// add n to the queue, or update it's position if F has changed
							// or if the heuristic h has become exact.
							deriveOrEstimateHValue(m, bm, im, t, n, bn, in);
							if (isFinal(bn, in) || getHScore(bn, in) > 0) {
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
		// Unable to sequentialize ILP solution obtained for the first marking in the net.
		// Add this solution to the undesirable set and try again.
		try {
			addUndesirableSolution();
			return null;
		} catch (LpSolveException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		alignmentResult &= ~Utils.OPTIMALALIGNMENT;
		alignmentResult |= Utils.FAILEDALIGNMENT;
		runTime = (int) ((System.nanoTime() - startTime) / 1000);
		return null;

	}

	protected void addUndesirableSolution() throws LpSolveException {

		byte[] solution = lpSolutions[0];
		debug.writeDebugInfo(Debug.DOT, "Adding undesirable solution: " + Arrays.toString(solution) + " h = "
				+ getHScore(0));

		// add a [0..1] slack variable
		double[] col = new double[numRows];
		solver.addColumn(col);
		numColumns++;
		solver.setUpbo(numColumns, 1);
		solver.setBinary(numColumns, true);

		double dy = 0;
		for (int t = randomRow.length - 1; t > 0;) {
			dy += randomRow[t] * solution[--t];
		}
		double c = dy + 100 * net.numTransitions();

		double[] row = Arrays.copyOf(randomRow, numColumns + 1);
		row[numColumns] = -c;
		// randomRow * solution - c * z < dy
		solver.addConstraint(row, LpSolve.LE, dy - 0.5);
		// randomRow * solution - c * z > dy - c
		solver.addConstraint(row, LpSolve.GE, dy - c + 0.5);
		numRows += 2;

		vars = new double[numColumns];
		blockedSolutions++;
	}

	protected double[] vars;

	@Override
	public int getExactHeuristic(int marking, int markingBlock, int markingIndex) {
		// start from correct right hand side
		long start = System.nanoTime();
		try {
			int lp = net.numPlaces();
			for (int p = lp; p-- > 0;) {
				// set right hand side to final marking 
				solver.setRh(lp, rhf[p]);
				if ((markingLo[markingBlock][markingIndex * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) != 0) {
					// adjust right hand side by - 1 from current
					solver.setRh(lp, solver.getRh(lp) - 1);
				}
				if ((markingHi[markingBlock][markingIndex * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) != 0) {
					// adjust right hand side by - 2 from current
					solver.setRh(lp, solver.getRh(lp) - 2);
				}
				lp--;
			}
			solver.setRh(targetFunctionRow + 1, Integer.MAX_VALUE);
			heuristicsComputed++;
			int solverResult = solver.solve();

			if (solverResult == LpSolve.INFEASIBLE || solverResult == LpSolve.NUMFAILURE) {
				// BVD: LpSolve has the tendency to give false infeasible or numfailure answers. 
				// It's unclear when or why this happens, but just in case...
				solver.defaultBasis();
				solverResult = solver.solve();

			}
			if (solverResult == LpSolve.OPTIMAL) {
				// retrieve the solution
				solver.getVariables(vars);
				//				System.out.println("Solved: " + Arrays.toString(vars));
				if (lpSolutions.length <= marking) {
					growArray(marking);
				}
				setNewLpSolution(marking, vars);

				// compute cost estimate
				double c = 0;
				for (short t = net.numTransitions(); t-- > 0;) {
					c += vars[t] * net.getCost(t);
				}

				if (c >= HEURISTICINFINITE) {
					alignmentResult |= Utils.HEURISTICFUNCTIONOVERFLOW;
					// continue with maximum heuristic value not equal to infinity.
					return HEURISTICINFINITE - 1;
				}

				// assume precision 1E-9 and round down
				return (int) (c + 1E-9);
			} else if (solverResult == LpSolve.INFEASIBLE) {
				return HEURISTICINFINITE;
			} else {
				//					lp.writeLp("D:/temp/antialignment/debugLP-Alignment.lp");
				System.err.println("Error code from LpSolve solver:" + solverResult);
				return HEURISTICINFINITE;
			}

		} catch (LpSolveException e) {
			return HEURISTICINFINITE;

		} finally {
			solveTime += System.nanoTime() - start;
		}

	}

	protected void growArray(int marking) {
		// grow lpSolutions by a single block at the time.
		lpSolutions = Arrays.copyOf(lpSolutions, marking + DEFAULTARRAYSIZE);
		lpSolutionsSize += DEFAULTARRAYSIZE * 8;
	}

	protected int getLpSolution(int marking, short transition) {
		if (marking == 0) {
			return lpSolutions[marking][transition] & 0xFF;
		} else {
			int pre = ((lpSolutions[marking][0] & 0xFF) << 24);
			pre |= ((lpSolutions[marking][1] & 0xFF) << 16);
			pre |= ((lpSolutions[marking][2] & 0xFF) << 8);
			pre |= (lpSolutions[marking][3] & 0xFF);
			int fired = ((lpSolutions[marking][4] & 0xFF) << 8);
			fired |= (lpSolutions[marking][5] & 0xFF);
			return getLpSolution(pre, transition) - (fired == transition ? 1 : 0);
		}

	}

	//	public byte[] getLpSolution(int marking) {
	//		return lpSolutions[marking];
	//	}
	//

	public void setNewLpSolution(int marking, double[] solution) {
		lpSolutions[marking] = new byte[net.numTransitions()];
		for (int i = lpSolutions[marking].length; i-- > 0;) {
			// round down into byte, but allow for precision up to 1E-9
			lpSolutions[marking][i] = (byte) ((int) (solution[i] + 1E-9));
		}
		lpSolutionsSize += 4 + lpSolutions[marking].length;
	}

	/**
	 * In ILP version, only one given final marking is the target.
	 */
	protected boolean isFinal(int marking) {
		return equalMarking(marking, net.getFinalMarking());
	}

	/**
	 * In ILP version, only one given final marking is the target.
	 */
	protected boolean isFinal(int block, int index) {
		return equalMarking(block, index, net.getFinalMarking());
	}

	protected void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to,
			int toBlock, int toIndex) {
		if (hasExactHeuristic(fromBlock, fromIndex) && getLpSolution(from, transition) >= 1) {

			// from Marking has exact heuristic
			// we can derive an exact heuristic from it
			if (to >= lpSolutions.length) {
				growArray(to);
			}
			setDerivedLpSolution(from, to, transition);
			// set the exact h score
			setHScore(toBlock, toIndex, getHScore(fromBlock, fromIndex) - net.getCost(transition), true);
			heuristicsDerived++;
		} else {
			assert false;
			// update hscore to higher values only
			if (getHScore(fromBlock, fromIndex) - net.getCost(transition) > getHScore(toBlock, toIndex)) {
				setHScore(toBlock, toIndex, getHScore(fromBlock, fromIndex) - net.getCost(transition), false);
			}
		}

	}

	protected void setDerivedLpSolution(int from, int to, short transition) {
		lpSolutions[to] = new byte[6];
		lpSolutions[to][0] = (byte) (from >>> 24);
		lpSolutions[to][1] = (byte) (from >>> 16);
		lpSolutions[to][2] = (byte) (from >>> 8);
		lpSolutions[to][3] = (byte) (from);
		lpSolutions[to][4] = (byte) (transition >>> 8);
		lpSolutions[to][5] = (byte) (transition);
		lpSolutionsSize += 4 + lpSolutions[to].length;
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
	public TObjectIntMap<Utils.Statistic> getStatistics() {
		TObjectIntMap<Statistic> map = super.getStatistics();
		map.put(Statistic.HEURISTICTIME, (int) (solveTime / 1000));
		return map;
	}

	@Override
	protected void writeEndOfAlignmentDot() {
		TObjectIntMap<Statistic> map = getStatistics();
		for (int m = 0; m < markingsReached; m++) {
			if (m > 0) {
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

	protected void terminateRun() {
		super.terminateRun();
		solver.deleteAndRemoveLp();
	}

	@Override
	protected boolean isEnabled(short transition, int block, int index) {
		// check enablement on the tokens and on the predecessor
		short preTransition = getPredecessorTransition(block, index);
		// allow firing only if allowed by the LpSolution.

		// STRICT VERSION
		if (getLpSolution(block * bm + index, transition) >= 1
				&& (preTransition <= transition || hasPlaceBetween(preTransition, transition))) {

			//		// LESS STRICT. DECISION ONLY ON COSTS.
			//		if (getHScore(block, index) >= net.getCost(transition)
			//				&& (preTransition <= transition || hasPlaceBetween(preTransition, transition))) {
			//		// DOESN"T WORK :)

			// allow firing only if there is a place between or if total order
			// is
			// respected
			byte[] input = net.getInput(transition);
			for (int i = bm; i-- > 0;) {
				// ((markingLo OR markingHi) AND input) should be input.
				if (((markingLo[block][bm * index + i] | markingHi[block][bm * index + i]) & input[i]) != input[i]) {
					return false;
				}
			}
			// Firing semantics do not allow to produce more than 3 tokens
			// in a place ((markingLo AND markingHi) AND output) should be 0.
			byte[] output = net.getOutput(transition);
			for (int i = bm; i-- > 0;) {
				if (((markingLo[block][bm * index + i] & markingHi[block][bm * index + i]) & output[i]) != 0) {
					// if violated, signal in alignmentResult and continue
					alignmentResult |= Utils.ENABLINGBLOCKEDBYOUTPUT;
					return false;
				}
			}
			return true;
		} else {
			// not allowed to fire
			return false;
		}
	}

	@Override
	protected void writeStatus() {
		debug.writeDebugInfo(Debug.NORMAL, "Markings reached:   " + String.format("%,d", markingsReached));
		debug.writeDebugInfo(Debug.NORMAL, "   FScore head:     " + getFScore(queue.peek()) + " = G: "
				+ getGScore(queue.peek()) + " + H: " + getHScore(queue.peek()));
		debug.writeDebugInfo(Debug.NORMAL, "   Queue size:      " + queue.size());
		debug.writeDebugInfo(Debug.NORMAL, "   Queue actions:   " + queueActions);
		debug.writeDebugInfo(Debug.NORMAL, "   Heuristics compu:" + heuristicsComputed);
		debug.writeDebugInfo(Debug.NORMAL, "   Heuristics deri: " + heuristicsDerived);
		//		debug.writeDebugInfo(Debug.NORMAL, "   Estimated memory:" + String.format("%,d", getEstimatedMemorySize()));
	}

}
