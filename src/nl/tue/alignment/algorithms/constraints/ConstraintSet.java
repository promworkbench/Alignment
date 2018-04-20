package nl.tue.alignment.algorithms.constraints;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.graphbased.directed.petrinet.PetrinetEdge;
import org.processmining.models.graphbased.directed.petrinet.PetrinetNode;
import org.processmining.models.graphbased.directed.petrinet.elements.Place;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;

import gnu.trove.map.TObjectShortMap;
import gnu.trove.map.TShortObjectMap;
import gnu.trove.map.hash.TObjectShortHashMap;
import gnu.trove.map.hash.TShortObjectHashMap;

public class ConstraintSet {

	private TShortObjectMap<Set<Constraint>> label2input = new TShortObjectHashMap<>();
	private TShortObjectMap<Set<Constraint>> label2output = new TShortObjectHashMap<>();
	private Set<Constraint> constraints = new HashSet<>();
	private String[] colNames;

	public ConstraintSet(Petrinet net, Marking initialMarking, XEventClasses classes, TObjectShortMap<XEventClass> c2id,
			TransEvClassMapping map) {

		int ts = net.getTransitions().size();
		int ps = net.getPlaces().size();
		int cs = c2id.size();

		int rows = cs + ts + ps;
		int columns = ts + cs + 1;

		short[][] matrix = new short[rows][columns];
		colNames = new String[columns];

		TObjectShortMap<Place> p2id = new TObjectShortHashMap<>(net.getPlaces().size(), 0.7f, (short) -1);
		TObjectShortMap<Transition> t2id = new TObjectShortHashMap<>(net.getTransitions().size(), 0.7f, (short) -1);

		// purely for consistent sorting
		for (Transition t : net.getTransitions()) {
			t2id.put(t, (short) t2id.size());
		}

		for (PetrinetEdge<? extends PetrinetNode, ? extends PetrinetNode> edge : net.getEdges()) {
			if (edge.getSource() instanceof Place) {
				short p = p2id.putIfAbsent((Place) edge.getSource(), (short) p2id.size());
				if (p < 0) {
					p = (short) (p2id.size() - 1);
				}
				short t = t2id.putIfAbsent((Transition) edge.getTarget(), (short) t2id.size());
				if (t < 0) {
					t = (short) (t2id.size() - 1);
				}
				XEventClass clazz = map.get(edge.getTarget());
				short c = clazz == null || ((Transition) edge.getTarget()).isInvisible() ? -1 : c2id.get(clazz);

				// p --> t[c]
				if (c >= 0) {
					// t is mapped to c.
					matrix[c][t] = 1;
					matrix[c][ts + c] = -1;
					// t occurs less than c
					matrix[cs + t][t] = -1;
					matrix[cs + t][ts + c] = 1;
					colNames[t] = clazz.toString().replace("+complete", "");
					colNames[ts + c] = clazz.toString().replace("+complete", "");
				}
				// t consumes from p
				matrix[cs + ts + p][t] -= 1;
				// initial marking
				matrix[cs + ts + p][ts + cs] = (short) -initialMarking.occurrences(edge.getSource());

			} else {
				short p = p2id.putIfAbsent((Place) edge.getTarget(), (short) p2id.size());
				if (p < 0) {
					p = (short) (p2id.size() - 1);
				}
				short t = t2id.putIfAbsent((Transition) edge.getSource(), (short) t2id.size());
				if (t < 0) {
					t = (short) (t2id.size() - 1);
				}
				XEventClass clazz = map.get(edge.getSource());
				short c = clazz == null || ((Transition) edge.getSource()).isInvisible() ? -1 : c2id.get(clazz);
				// t[c] --> p

				if (c >= 0) {
					// t is mapped to c.
					matrix[c][t] = 1;
					matrix[c][ts + c] = -1;
					// t occurs less than c
					matrix[cs + t][t] = -1;
					matrix[cs + t][ts + c] = 1;
					colNames[t] = clazz.toString().replace("+complete", "");
					colNames[ts + c] = clazz.toString().replace("+complete", "");
				}
				// t produces in p
				matrix[cs + ts + p][t] += 1;
				// initial marking
				matrix[cs + ts + p][ts + cs] = (short) -initialMarking.occurrences(edge.getTarget());

			}

		}

		//		System.out.println("Before:");
		//		printMatrix(matrix);

		int[] first1 = new int[cs];
		for (int r = 0; r < cs; r++) {
			first1[r] = ts;
			for (int c = 0; c < ts && first1[r] == ts; c++) {
				if (matrix[r][c] == 1) {
					first1[r] = c;
				}
			}
		}

		int lastStrong = ps - 1;
		boolean done;
		// now matrix needs to be swept to create 0 columns in the lower left part.
		for (int c = 0; c < ts; c++) {

			short[][] newMatrix = new short[ps][];

			//			System.out.println("Column " + c);
			// try to reduce the lower elements of column c to 0, by
			// 1) subtracting or adding rows 0..cs-1
			// 2) adding a row from cs+ts..rows-1
			// 3) adding rows cs..cs+ts-1
			// without introducing non-zero elements in earlier columns
			for (int r = cs + ts; r < rows; r++) {
				// copy original
				int rn = r - cs - ts;
				newMatrix[rn] = Arrays.copyOf(matrix[r], matrix[r].length);

				done = matrix[r][c] == 0;
				//				if (!done) {
				//					System.out.println("Row " + r);
				//				}
				if (matrix[r][c] > 0) {

					// element at row r > 0
					// reduce by subtracting and element from row 0..cs-1
					for (int s = 0; s < cs && !done; s++) {
						if (first1[s] == c) {
							// subtract this row
							short f1 = matrix[r][c];
							for (int x = c; x < columns; x++) {
								newMatrix[rn][x] -= f1 * matrix[s][x];
							}
							done = true;
						}
					}

					// if not done, try to add another constraint
					for (int s = cs + ts; s < rows && !done; s++) {
						if (matrix[s][c] < 0) {
							short f1 = matrix[r][c];
							short f2 = matrix[s][c];
							// we can use row s, but we have to find the least common multiple
							for (int x = c; x < columns; x++) {
								newMatrix[rn][x] *= -f2;
								newMatrix[rn][x] += f1 * matrix[s][x];
							}
							done = true;
						}
					}

					// finally, try to add a weakening constraint
					for (int s = cs; s < cs + ts && !done; s++) {
						if (matrix[s][c] == -1) {
							short f1 = matrix[r][c];
							// we can use row s, but we have to find the least common multiple
							for (int x = c; x < columns; x++) {
								newMatrix[rn][x] += f1 * matrix[s][x];
							}
							done = true;

							// swap with lastStrong to avoid over-use of weak constraints
							short[] tmp = newMatrix[lastStrong];
							newMatrix[lastStrong] = newMatrix[rn];
							newMatrix[rn] = tmp;
							// move last strong pointer up
							lastStrong--;
							// decrease r, as the new row might have a non 0 value at [r][c]
							r--;
						}
					}

					// if all else fails, we have a positive value left on matrix[r][c] and no way to
					// reduce it, even by weakening. This implies a tau-transition, remove the row.
					if (!done) {
						// eliminate row r;
						Arrays.fill(newMatrix[rn], (short) 0);

						// swap with lastStrong to avoid over-use of weak constraints
						short[] tmp = newMatrix[lastStrong];
						newMatrix[lastStrong] = newMatrix[rn];
						newMatrix[rn] = tmp;
						// move last strong pointer up
						lastStrong--;
						// decrease r, as the new row might have a non 0 value at [r][c]
						r--;
					}

				} else if (matrix[r][c] < 0) {

					// element at row r < 0
					// reduce by adding and element from row 0..cs-1
					for (int s = 0; s < cs && !done; s++) {
						if (first1[s] == c) {
							// add this row
							short f1 = matrix[r][c];
							for (int x = c; x < columns; x++) {
								newMatrix[rn][x] -= f1 * matrix[s][x];
							}
							done = true;
						}
					}

					// if not done, try to add another constraint
					for (int s = cs + ts; s < rows && !done; s++) {
						if (matrix[s][c] > 0) {
							short f1 = matrix[r][c];
							short f2 = matrix[s][c];
							// we can use row s, but we have to find the least common multiple
							for (int x = c; x < columns; x++) {
								newMatrix[rn][x] *= f2;
								newMatrix[rn][x] -= f1 * matrix[s][x];
							}
							done = true;
						}
					}

					// if not done, set to 0 as this is only weakens the constraint
					if (!done) {
						newMatrix[rn][c] = 0;
						// swap with lastStrong to avoid over-use of weak constraints
						short[] tmp = newMatrix[lastStrong];
						newMatrix[lastStrong] = newMatrix[rn];
						newMatrix[rn] = tmp;
						// move last strong pointer up
						lastStrong--;
						// decrease r, as the new row might have a non 0 value at [r][c]
						r--;
					}
				}
			} // for rows
				// copy derived back in.
			for (int r = cs + ts; r < rows; r++) {
				// copy original
				matrix[r] = newMatrix[r - cs - ts];
			}
		} // for columns

		//		System.out.println("After:");
		//		printMatrix(matrix);

		for (short c = 0; c < c2id.size(); c++) {
			label2input.put(c, new HashSet<Constraint>());
			label2output.put(c, new HashSet<Constraint>());
		}

		String[] classColNames = Arrays.copyOfRange(colNames, ts, ts + cs);
		// now translate the matrix to constraints per label
		for (int r = cs + ts; r < rows; r++) {
			Constraint constraint = new Constraint(c2id.size(), matrix[r][columns - 1], classColNames);
			for (int c = ts; c < ts + cs; c++) {
				if (matrix[r][c] > 0) {
					constraint.addInput((short) (c - ts), matrix[r][c]);
				} else if (matrix[r][c] < 0) {
					constraint.addOutput((short) (c - ts), -matrix[r][c]);
				}
			}
			if (constraints.add(constraint)) {
				// new constraint;
				for (int c = ts; c < ts + cs; c++) {
					if (matrix[r][c] > 0) {
						label2input.get((short) (c - ts)).add(constraint);
					} else if (matrix[r][c] < 0) {
						label2output.get((short) (c - ts)).add(constraint);
					}
				}
			}
		}

		//		System.out.println("Found " + constraints.size() + " constraints.");

	}

	private void printMatrix(short[][] matrix) {

		for (int c = 0; c < colNames.length; c++) {
			System.out.print(colNames[c]);
			System.out.print(",");
		}
		System.out.println();
		for (int c = 0; c < colNames.length; c++) {
			System.out.print("" + c);
			System.out.print(",");
		}
		System.out.println();

		for (int r = 0; r < matrix.length; r++) {
			for (int c = 0; c < matrix[r].length; c++) {
				System.out.print(matrix[r][c]);
				System.out.print(",");
			}
			System.out.println();
		}
	}

	public void reset() {
		for (Constraint constraint : constraints) {
			constraint.reset();
		}
	}

	public boolean satisfiedAfterOccurence(short label) {
		// process all relevant constraints for internal state consistency
		boolean satisfied = true;
		for (Constraint constraint : label2input.get(label)) {
			satisfied &= constraint.satisfiedAfterOccurence(label);
		}
		for (Constraint constraint : label2output.get(label)) {
			satisfied &= constraint.satisfiedAfterOccurence(label);
		}
		return satisfied;
	}

}
