package nl.tue.alignment.algorithms.syncproduct;

import java.util.Arrays;
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
import nl.tue.alignment.algorithms.datastructures.Constraint;

public class ConstraintSet {

	private TShortObjectMap<Set<Constraint>> label2input = new TShortObjectHashMap<>();
	private TShortObjectMap<Set<Constraint>> label2output = new TShortObjectHashMap<>();

	public ConstraintSet(Petrinet net, Marking initialMarking, XEventClasses classes, TObjectShortMap<XEventClass> c2id,
			TransEvClassMapping map) {

		int ts = net.getTransitions().size();
		int ps = net.getPlaces().size();
		int cs = c2id.size() + 2;

		int rows = cs + ts + ps;
		int columns = ts + cs + 1;

		short[][] matrix = new short[rows][columns];

		TObjectShortMap<Place> p2id = new TObjectShortHashMap<>(net.getPlaces().size(), 0.7f, (short) -1);
		TObjectShortMap<Transition> t2id = new TObjectShortHashMap<>(net.getTransitions().size(), 0.7f, (short) -1);
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
				short c = (short) (clazz == null || ((Transition) edge.getTarget()).isInvisible() ? ts - 2
						: c2id.get(clazz));
				// p --> t[c]
				// t is mapped to c.
				matrix[c][t] = 1;
				matrix[c][ts + c] = -1;
				// t occurs less than c
				matrix[cs + t][t] = -1;
				matrix[cs + t][ts + c] = 1;
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
				short c = (short) (clazz == null || ((Transition) edge.getSource()).isInvisible() ? ts - 1
						: c2id.get(clazz));
				// t[c] --> p

				// t is mapped to c.
				matrix[c][t] = 1;
				matrix[c][ts + c] = -1;
				// t occurs less than c
				matrix[cs + t][t] = -1;
				matrix[cs + t][ts + c] = 1;
				// t produces in p
				matrix[cs + ts + p][t] += 1;
				// initial marking
				matrix[cs + ts + p][ts + cs] = (short) -initialMarking.occurrences(edge.getTarget());

			}

		}

		System.out.println("Before:");
		printMatrix(matrix);

		int[] first1 = new int[cs];
		for (int r = 0; r < cs; r++) {
			first1[r] = ts;
			for (int c = 0; c < ts && first1[r] == ts; c++) {
				if (matrix[r][c] == 1) {
					first1[r] = c;
				}
			}
		}

		boolean done;
		// now matrix needs to be swept to create 0 columns in the lower left part.
		for (int c = 0; c < ts; c++) {
			//			System.out.println("Column " + c);
			// try to reduce the lower elements of column c to 0, by
			// 1) subtracting or adding rows 0..cs-1
			// 2) adding a row from cs+ts..rows-1
			// 3) adding rows cs..cs+ts-1
			// without introducing non-zero elements in earlier columns
			for (int r = cs + ts; r < rows; r++) {
				done = matrix[r][c] == 0;
				//				if (!done) {
				//					System.out.println("Row " + r);
				//				}
				if (matrix[r][c] > 0) {
					assert !done;
					// element at row r > 0
					// reduce by subtracting and element from row 0..cs-1
					for (int s = 0; s < cs && !done; s++) {
						if (first1[s] == c) {
							// subtract this row
							short f1 = matrix[r][c];
							for (int x = c; x < columns; x++) {
								matrix[r][x] -= f1 * matrix[s][x];
							}
							done = true;
						}
					}
					assert !done || matrix[r][c] == 0;
					// if not done, try to add another constraint
					for (int s = cs + ts; s < rows && !done; s++) {
						if (matrix[s][c] < 0) {
							short f1 = matrix[r][c];
							short f2 = matrix[s][c];
							// we can use row s, but we have to find the least common multiple
							for (int x = c; x < columns; x++) {
								matrix[r][x] *= -f2;
								matrix[r][x] += f1 * matrix[s][x];
							}
							done = true;
						}
					}
					assert !done || matrix[r][c] == 0;
					// finally, try to add a diminishing constraint
					for (int s = cs; s < cs + ts && !done; s++) {
						if (matrix[s][c] == -1) {
							short f1 = matrix[r][c];
							// we can use row s, but we have to find the least common multiple
							for (int x = c; x < columns; x++) {
								matrix[r][x] += f1 * matrix[s][x];
							}
							done = true;
						}
					}
					assert !done || matrix[r][c] == 0;
					//					System.out.println("Reduced row " + r + "column " + c);
					//					printMatrix(matrix);
					//					System.out.println();
				} else if (matrix[r][c] < 0) {
					assert !done;
					// element at row r < 0
					// reduce by adding and element from row 0..cs-1
					for (int s = 0; s < cs && !done; s++) {
						if (first1[s] == c) {
							// add this row
							short f1 = matrix[r][c];
							for (int x = c; x < columns; x++) {
								matrix[r][x] -= f1 * matrix[s][x];
							}
							done = true;
						}
					}
					assert !done || matrix[r][c] == 0;
					// if not done, try to add another constraint
					for (int s = cs + ts; s < rows && !done; s++) {
						if (matrix[s][c] > 0) {
							short f1 = matrix[r][c];
							short f2 = matrix[s][c];
							// we can use row s, but we have to find the least common multiple
							for (int x = c; x < columns; x++) {
								matrix[r][x] *= f2;
								matrix[r][x] -= f1 * matrix[s][x];
							}
							done = true;
						}
					}
					assert !done || matrix[r][c] == 0;
					//					System.out.println("Reduced row " + r + "column " + c);
					//					printMatrix(matrix);
					//					System.out.println();
				}
				if (!done || matrix[r][c] != 0) {
					System.out.println(
							"Could not reduce column " + c + " to 0, got stuck on row +" + r + " done: " + done);
					// eliminate row r;
					Arrays.fill(matrix[r], (short) 0);
				} else {
					assert matrix[r][c] == 0;
				}
			}
		}

		System.out.println("After:");
		printMatrix(matrix);

	}

	private void printMatrix(short[][] matrix) {
		for (int r = 0; r < matrix.length; r++) {
			for (int c = 0; c < matrix[r].length; c++) {
				System.out.print(matrix[r][c]);
				System.out.print(",");
			}
			System.out.println();
		}
	}

}
