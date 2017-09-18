package nl.tue.alignment.algorithms;

import gnu.trove.iterator.TIntIterator;
import gnu.trove.iterator.TIntShortIterator;
import gnu.trove.list.linked.TIntLinkedList;
import gnu.trove.map.TIntShortMap;
import gnu.trove.map.hash.TIntShortHashMap;

import java.util.Arrays;

import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.SyncProduct;
import nl.tue.alignment.Utils;
import nl.tue.astar.util.ilp.LPMatrixException;

public class BackTrackingAStar extends AStar {

	private static final int DEPTH = 5;
	private LpSolve tempSolver;
	private boolean useInt;

	public BackTrackingAStar(SyncProduct product, boolean useInt, Debug debug) throws LPMatrixException {
		super(product, true, true, true, useInt, debug);
		this.useInt = useInt;

	}

	/**
	 * Get the exact heuristic for a state that is currently estimated
	 * 
	 * @param marking
	 * @param markingBlock
	 * @param markingIndex
	 * @return
	 */
	protected int getExactHeuristicForEstimated(int marking, int markingBlock, int markingIndex) {
		// marking currently has an inexact, estimated heuristic. Implying that a true heuristic could not be derived.
		debug.writeDebugInfo(Debug.NORMAL, "Flushing m" + marking + " : " + Utils.asBag(getMarking(marking), net));
		// compute the exact heuristic for this marking.

		tempSolver = solver;
		try {
			solver = solver.copyLp();
			pushUpFirstStep(marking, markingBlock, markingIndex, numCols, numRows + 1);
		} catch (LpSolveException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} finally {
			solver.deleteAndRemoveLp();
			solver = tempSolver;
			vars = new double[net.numTransitions()];
		}

		return getHScore(markingBlock, markingIndex);
	}

	//	private void pushUp(int marking, int mb, int mi, int cols, int rows) throws LpSolveException {
	//		if (marking != NOPREDECESSOR) {
	//
	//			int[] place2row = new int[net.numPlaces()];
	//			int[] trans2col = new int[net.numTransitions()];
	//			TIntList newMatrix = new TIntArrayList(10);
	//			int xtraCols = 0;
	//			int xtraRows = 0;
	//
	//			// one transition needs to be 1.
	//			solver.addConstraint(new double[numCols], LpSolve.EQ, 1);
	//			col2transition.add(-1);
	//			// extend solver with requirement to fire one of the enabled transitions.
	//			for (short t = 0; t < net.numTransitions(); t++) {
	//				if (isEnabled(t, mb, mi)) {
	//					if (trans2col[t] == 0) {
	//						// create extra column
	//						trans2col[t] = cols + xtraCols;
	//						xtraCols++;
	//						solver.addColumn(tempSolver.getPtrColumn(t + 1));
	//						solver.setColName(trans2col[t], net.getTransitionLabel(t));
	//						solver.setInt(trans2col[t], useInt);
	//						solver.setUpbo(trans2col[t], 255);
	//						solver.setObj(trans2col[t], net.getCost(t));
	//						solver.setMat(rows, trans2col[t], 1);
	//						col2transition.add(t);
	//					}
	//					byte[] input = net.getInput(t);
	//					for (short p = net.numPlaces(); p-- > 0;) {
	//						if ((input[p >>> 3] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0) {
	//							// Place p serves as input for t
	//							if (place2row[p] == 0) {
	//								// create extra column
	//								place2row[p] = rows + xtraRows + 1;
	//								xtraRows++;
	//								int tokensRequired = 0;
	//								tokensRequired += (markingLo[mb][mi * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0 ? 1
	//										: 0;
	//								tokensRequired += (markingHi[mb][mi * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0 ? 2
	//										: 0;
	//
	//								solver.addConstraint(new double[cols + xtraCols], LpSolve.GE, -tokensRequired);
	//								solver.setRowName(place2row[p], net.getPlaceLabel(p));
	//							}
	//							newMatrix.add((p << 16) | t);
	//						}
	//					}
	//				}
	//			}
	//			// The newMatrix contains all pairs of transition/place with -1 in the 
	//			// matrix to be added.
	//			for (int i = 0; i < newMatrix.size(); i++) {
	//				short p = (short) (newMatrix.get(i) >>> 16);
	//				short t = (short) (newMatrix.get(i));
	//				solver.setMat(place2row[p], trans2col[t], -1);
	//			}
	//			solver.printLp();
	//			vars = new double[cols + xtraCols];
	//			getExactHeuristic(marking, mi, mb);
	//
	//			int predecessor = getPredecessor(mb, mi);
	//			pushUp(predecessor, predecessor >>> blockBit, predecessor & blockMask, cols + xtraCols, rows + xtraRows);
	//			col2transition.remove(cols, xtraCols);
	//		}
	//
	//	}

	private void pushUpFirstStep(int marking, int mb, int mi, int cols, int rows) throws LpSolveException {
		if (marking == NOPREDECESSOR) {
			return;
		}
		// Duplicate the matrix in the solver
		for (short t = 0; t < net.numTransitions(); t++) {
			solver.addColumn(solver.getPtrColumn(t + 1));
			if (debug != Debug.NONE) {
				//				solver.setColName(cols + t, "y1_" + net.getTransitionLabel(t));
			}
			solver.setInt(cols + t, useInt);
			solver.setUpbo(cols + t, 1);
			solver.setObj(cols + t, net.getCost(t));
		}

		double[] row = new double[cols + net.numTransitions()];
		for (short p = 0; p < net.numPlaces(); p++) {
			int tokensRequired = 0;
			tokensRequired += (markingLo[mb][mi * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0 ? 1 : 0;
			tokensRequired += (markingHi[mb][mi * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0 ? 2 : 0;
			solver.addConstraint(row, LpSolve.GE, -tokensRequired);
			if (debug != Debug.NONE) {
				solver.setRowName(rows + p, "y1_" + net.getPlaceLabel(p));
			}
		}
		// at most one of the new transitions can fire
		for (int t = net.numTransitions() + cols; t-- > cols;) {
			row[t] = 1;
		}
		solver.addConstraint(row, LpSolve.LE, 1);
		if (debug != Debug.NONE) {
			solver.setRowName(rows + net.numPlaces(), "1xy1_LE_1");
		}
		// only allow for x if |y|==1;
		double constant = -(1 / (net.numTransitions() * 255.0));
		for (int t = cols; t-- > 0;) {
			row[t] = constant;
		}
		solver.addConstraint(row, LpSolve.GE, 0);
		if (debug != Debug.NONE) {
			solver.setRowName(rows + net.numPlaces() + 1, "y1_GE_x/C");
		}
		for (short t = net.numTransitions(); t-- > 0;) {
			byte[] input = net.getInput(t);
			for (short p = net.numPlaces(); p-- > 0;) {
				if ((input[p >>> 3] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0) {
					// place p is input for transition t
					solver.setMat(rows + p, cols + t, -1);
				}
			}
		}
		//		solver.printLp();

		// Solver is now set up to compute a approximation based on 1 exact step and the rest 
		// approximate steps. (in contrast to 0 exact steps by default)
		handleHeuristicUpdate(marking, mb, mi, cols, rows, 1);
	}

	private void handleHeuristicUpdate(int marking, int mb, int mi, int cols, int rows, int n) throws LpSolveException {

		// Check if there are successor markings of this marking still in the open set. 
		// otherwise there's no point
		TIntShortMap successors = new TIntShortHashMap(16);
		for (short t = net.numTransitions(); n > 1 && t-- > 0;) {
			if (isEnabled(t, mb, mi)) {
				// transition t is enabled
				byte[] successor = fire(t, mb, mi);
				int number = visited.add(successor, block * blockSize + indexInBlock);
				if (!isClosed(number)) {
					successors.put(number, t);
				}
			}
		}

		int heuristic = getHScore(mb, mi);
		if (n == 1 || !successors.isEmpty()) {

			vars = new double[solver.getNcolumns()];
			debug.writeDebugInfo(Debug.NORMAL,
					"Computed heuristic with precision " + n + " for Marking " + Utils.asBag(getMarking(marking), net)
							+ " H^ = " + getHScore(mb, mi));
			heuristic = getExactHeuristic(marking, mb, mi);
			debug.writeDebugInfo(Debug.NORMAL, " H = " + heuristic);

		}

		int prevHScore = getHScore(mb, mi);

		//TODO: Handle infeasibility

		if (heuristic < prevHScore) {
			// Can only be that there is already a better heuristic. No need to proceed.
			return;
		}
		// potential improvement. Improve heuristic
		// not only on this state, but on entire subtree
		setHScore(mb, mi, heuristic, true);

		// update the successor states where relevant
		TIntShortIterator iterator = successors.iterator();
		while (iterator.hasNext()) {
			iterator.advance();
			int successor = iterator.key();
			short transition = iterator.value();
			if (getHScore(successor) < heuristic - net.getCost(transition)) {
				deriveOrEstimateHValue(marking, mb, mi, transition, successor, successor >>> blockBit, successor
						& blockMask);
				debug.writeDebugInfo(
						Debug.NORMAL,
						"    updated heuristic at precision " + n + " for Marking "
								+ Utils.asBag(getMarking(successor), net) + " H = " + getHScore(successor));

			}
		}

		if (successors.isEmpty() || heuristic > prevHScore) {
			assert n == 1 || hasExactHeuristic(mb, mi);

			int predecessor = getPredecessor(mb, mi);
			pushUpNextStep(predecessor, predecessor >>> blockBit, predecessor & blockMask, solver.getNcolumns() + 1,
					solver.getNrows() + 1, n + 1);

		} else {
			setHScore(mb, mi, heuristic, true);
		}

	}

	private void pushUpNextStep(int marking, int mb, int mi, int cols, int rows, int n) throws LpSolveException {
		if (marking == NOPREDECESSOR || n > DEPTH) {
			return;
		}
		// Duplicate the matrix in the solver
		for (short t = 0; t < net.numTransitions(); t++) {
			double[] orgColumn = solver.getPtrColumn(t + 1);
			Arrays.fill(orgColumn, net.numPlaces() + 1, orgColumn.length, 0);
			solver.addColumn(orgColumn);
			if (debug != Debug.NONE) {
				solver.setColName(cols + t, "y" + n + "_" + net.getTransitionLabel(t));
			}
			solver.setInt(cols + t, useInt);
			solver.setUpbo(cols + t, 1);
			solver.setObj(cols + t, net.getCost(t));
		}
		double[] row = new double[cols + net.numTransitions()];
		for (short p = 0; p < net.numPlaces(); p++) {
			int tokensRequired = 0;
			tokensRequired += (markingLo[mb][mi * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0 ? 1 : 0;
			tokensRequired += (markingHi[mb][mi * bm + (p >>> 3)] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0 ? 2 : 0;
			int r = rows + p;
			solver.addConstraint(row, LpSolve.GE, -tokensRequired);
			if (debug != Debug.NONE) {
				solver.setRowName(r, "y" + n + "_" + net.getPlaceLabel(p));
			}
			do {
				solver.setRh(r, -tokensRequired);
				// adjust the marking on all rh parts
				r -= net.numPlaces() + 2;
			} while (r > net.numPlaces());
		}
		// at most one of the new transitions can fire
		for (int t = net.numTransitions() + cols; t-- > cols;) {
			row[t] = 1;
		}
		solver.addConstraint(row, LpSolve.LE, 1);
		if (debug != Debug.NONE) {
			solver.setRowName(rows + net.numPlaces(), "1xy" + n + "_LE_1");
		}
		// only allow for y_n if |y_(n-1)|==1;
		for (int t = cols; t-- > cols - net.numTransitions();) {
			row[t] = -1;
		}
		solver.addConstraint(row, LpSolve.GE, 0);
		if (debug != Debug.NONE) {
			solver.setRowName(rows + net.numPlaces() + 1, "y" + n + "_LE_y" + (n - 1));
		}
		for (short t = net.numTransitions(); t-- > 0;) {
			byte[] input = net.getInput(t);
			for (short p = net.numPlaces(); p-- > 0;) {
				if ((input[p >>> 3] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0) {
					// place p is input for transition t
					int c = cols + t;
					do {
						solver.setMat(rows + p, c, -1);
						c -= net.numTransitions();
					} while (c > net.numTransitions());
				}
			}
			byte[] output = net.getOutput(t);
			for (short p = net.numPlaces(); p-- > 0;) {
				if ((output[p >>> 3] & (Utils.BYTEHIGHBIT >>> (p & 7))) > 0) {
					// place p is input for transition t
					int c = cols + t - net.numTransitions();
					do {
						solver.setMat(rows + p, c, solver.getMat(rows + p, c) + 1);
						c -= net.numTransitions();
					} while (c > net.numTransitions());
				}
			}
		}
		//		solver.printLp();

		handleHeuristicUpdate(marking, mb, mi, cols, rows, n);

	}

	@Override
	protected double computeCostForVars() {
		double c = 0;
		for (int t = vars.length; t-- > 0;) {
			c += vars[t] * net.getCost((short) (t % net.numTransitions()));
		}
		return c;
	}

	@Override
	protected void setNewLpSolution(int marking, double[] solution) {
		lpSolutions[marking] = new byte[numCols + 1];
		lpSolutions[marking][0] = COMPUTED;
		for (int i = solution.length; i-- > 0;) {
			// round down into byte, but allow for precision up to 1E-9
			lpSolutions[marking][1 + i % net.numTransitions()] = (byte) ((int) (solution[i] + 1E-9));
		}
		lpSolutionsSize += 4 + lpSolutions[marking].length;
	}

	@Override
	protected void initializeEstimated() {
		estimatedList = new TIntLinkedList();
	}

	private TIntLinkedList estimatedList;

	@Override
	protected void addToEstimated(int marking, int gScore, int hScore) {
		if (estimatedList.isEmpty()) {
			estimatedList.add(marking);
		} else {
			estimatedList.insert(0, marking);
		}
	}

	@Override
	protected boolean estimatedEmpty() {
		return estimatedList.isEmpty();
	}

	@Override
	protected int flushEstimatedToQueue() {
		int currentMinimumFScore;
		//		if (queue.isEmpty()) {
		currentMinimumFScore = currentFScore;
		//		} else {
		//			currentMinimumFScore = getFScore(queue.peek());
		//		}
		if (estimatedList.isEmpty()) {
			// no estimated heuristics
			currentFScore = getFScore(queue.peek());
			pollActions++;
			return queue.poll();
		}
		flushActions++;
		if (estimatedList.size() > maxEstimated) {
			maxEstimated = estimatedList.size();
		}

		// for each estimated heuristic, we update.
		do {
			TIntIterator it = estimatedList.iterator();

			int listMin = Integer.MAX_VALUE;
			while (it.hasNext()) {
				int estimated = it.next();
				// if closed, ignore.
				if (isClosed(estimated)) {
					it.remove();
					continue;
				}
				int eb = estimated >>> blockBit;
				int ei = estimated & blockMask;
				// if not closed and F-score is minimal
				if (getFScore(eb, ei) == currentMinimumFScore) {
					it.remove();
					int heuristic = getExactHeuristicForEstimated(estimated, eb, ei);
					setHScore(eb, ei, heuristic, true);
					if (getFScore(eb, ei) == currentMinimumFScore) {
						return estimated;
					} else {
						// TODO: What if F-score increases due to later updates?
						queue.add(estimated);
						queueActions++;
					}
				} else if (getFScore(eb, ei) < listMin) {
					listMin = getFScore(eb, ei);
				}
			}
			if (!queue.isEmpty() && getFScore(queue.peek()) < listMin) {
				pollActions++;
				return queue.poll();
			}
			currentMinimumFScore = listMin;
		} while (!estimatedList.isEmpty());
		return queue.poll();

	}
}
