package nl.tue.alignment.algorithms.syncproduct;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;

import gnu.trove.list.TByteList;
import gnu.trove.list.TIntList;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import nl.tue.alignment.algorithms.syncproduct.petrinet.ReducedPetriNet;
import nl.tue.alignment.algorithms.syncproduct.petrinet.ReducedPlace;
import nl.tue.alignment.algorithms.syncproduct.petrinet.ReducedTransition;

public class ReducedSyncProductFactory {

	private TObjectIntMap<XEventClass> c2id;
	private ReducedPetriNet reducedNet;
	private TObjectIntMap<Transition> trans2id;
	private XEventClasses classes;

	private List<String> transitionLabels = new ArrayList<>();
	private List<String> placeLabels = new ArrayList<>();
	private TIntList eventNumbers = new TIntArrayList();
	private TIntList ranks = new TIntArrayList();
	private TByteList types = new TByteArrayList();
	private TIntList moves = new TIntArrayList();
	private TIntList cost = new TIntArrayList();

	public ReducedSyncProductFactory(Petrinet net, XEventClasses classes, TObjectIntMap<XEventClass> c2id,
			TransEvClassMapping map, Marking initialMarking, Marking finalMarking) {
		this(net, classes, c2id, map, new GenericMap2Int<Transition>(1), new GenericMap2Int<XEventClass>(1), //
				new GenericMap2Int<Transition>(0), initialMarking, finalMarking);
	}

	public ReducedSyncProductFactory(Petrinet net, XEventClasses classes, TObjectIntMap<XEventClass> c2id,
			TransEvClassMapping map, Map<Transition, Integer> mapTrans2Cost, Map<XEventClass, Integer> mapEvClass2Cost,
			Map<Transition, Integer> mapSync2Cost, Marking initialMarking, Marking finalMarking) {
		this(net, classes, c2id, map, new GenericMap2Int<>(mapTrans2Cost, 1), new GenericMap2Int<>(mapEvClass2Cost, 1), //
				new GenericMap2Int<>(mapSync2Cost, 0), initialMarking, finalMarking);
	}

	public ReducedSyncProductFactory(Petrinet net, XEventClasses classes, TObjectIntMap<XEventClass> c2id,
			TransEvClassMapping map, TObjectIntMap<Transition> mapTrans2Cost,
			TObjectIntMap<XEventClass> mapEvClass2Cost, TObjectIntMap<Transition> mapSync2Cost, Marking initialMarking,
			Marking finalMarking) {
		this(net, classes, c2id, map, new GenericMap2Int<>(mapTrans2Cost, 1), new GenericMap2Int<>(mapEvClass2Cost, 1), //
				new GenericMap2Int<>(mapSync2Cost, 0), initialMarking, finalMarking);
	}

	public ReducedSyncProductFactory(Petrinet net, XEventClasses classes, TObjectIntMap<XEventClass> c2id,
			TransEvClassMapping map, Map<Transition, Integer> mapTrans2Cost, Map<XEventClass, Integer> mapEvClass2Cost,
			Marking initialMarking, Marking finalMarking) {
		this(net, classes, c2id, map, new GenericMap2Int<>(mapTrans2Cost, 1), new GenericMap2Int<>(mapEvClass2Cost, 1), //
				new GenericMap2Int<Transition>(0), initialMarking, finalMarking);
	}

	public ReducedSyncProductFactory(Petrinet net, XEventClasses classes, TObjectIntMap<XEventClass> c2id,
			TransEvClassMapping map, TObjectIntMap<Transition> mapTrans2Cost,
			TObjectIntMap<XEventClass> mapEvClass2Cost, Marking initialMarking, Marking finalMarking) {
		this(net, classes, c2id, map, new GenericMap2Int<>(mapTrans2Cost, 1), new GenericMap2Int<>(mapEvClass2Cost, 1), //
				new GenericMap2Int<Transition>(0), initialMarking, finalMarking);
	}

	private ReducedSyncProductFactory(Petrinet net, XEventClasses classes, TObjectIntMap<XEventClass> c2id,
			TransEvClassMapping map, GenericMap2Int<Transition> mapTrans2Cost,
			GenericMap2Int<XEventClass> mapEvClass2Cost, GenericMap2Int<Transition> mapSync2Cost,
			Marking initialMarking, Marking finalMarking) {

		this.c2id = c2id;

		// setup internal structures
		trans2id = new TObjectIntHashMap<>();
		int i = 0;
		for (Transition t : net.getTransitions()) {
			trans2id.put(t, i);
			i++;
		}

		// produce a Petri net for reduction
		reducedNet = new ReducedPetriNet(net, classes, trans2id, c2id, map, mapTrans2Cost, mapEvClass2Cost,
				mapSync2Cost, initialMarking, finalMarking);

		// reduce the net to a minimum
		reducedNet.reduce(Integer.MAX_VALUE, 2);

		//		PrintStream writer;
		//		try {
		//			i = 0;
		//			int step = 20;
		//			do {
		//				writer = new PrintStream(new File(String.format("c://temp//dot//model%03d.dot", i)));
		//				reducedNet.toDot(writer);
		//				writer.close();
		//				i += step;
		//			} while (reducedNet.reduce(step, 2));
		//			writer = new PrintStream(new File(String.format("c://temp//dot//model%03d.dot", i)));
		//			reducedNet.toDot(writer);
		//			writer.close();
		//
		//		} catch (FileNotFoundException e) {
		//			// TODO Auto-generated catch block
		//			e.printStackTrace();
		//		}
		//
		//		System.exit(0);
		// find transitions with identical input/output

		// start reducing the model into a new model applying as many rules as possible.

		// prepare Data Structures for synchronous product.
		for (ReducedTransition t : reducedNet.getTransitions()) {
			// transition label
			transitionLabels.add(t.toIdString());
		}
		for (ReducedPlace p : reducedNet.getPlaces()) {
			placeLabels.add(p.toIdString());
		}
		

		//		SyncProductImpl(String label, int numClasses, String[] transitions, String[] places, int[] eventNumbers,int[] ranks, byte[] types, int[] moves, int[] cost)

	}

	//	public synchronized SyncProduct getSyncProduct(XTrace xTrace, List<Transition> transitionList,
	//			boolean partiallyOrderSameTimestamp) {
	//		String traceLabel = XConceptExtension.instance().extractName(xTrace);
	//		if (traceLabel == null) {
	//			traceLabel = "XTrace@" + Integer.toHexString(xTrace.hashCode());
	//		}
	//		if (partiallyOrderSameTimestamp) {
	//			PartiallyOrderedTrace trace = getPartiallyOrderedTrace(xTrace, traceLabel);
	//			// Do the ranking on this trace.
	//			return getPartiallyOrderedSyncProduct(trace, transitionList);
	//		} else {
	//			return getLinearSyncProduct(getLinearTrace(xTrace, traceLabel), transitionList);
	//		}
	//
	//	}
	//

	/**
	 * @param transitionList
	 * @return
	 */
//	public synchronized SyncProduct getSyncProductForEmptyTrace() {
//	}

}
