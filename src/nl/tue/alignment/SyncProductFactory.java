package nl.tue.alignment;

import gnu.trove.list.TIntList;
import gnu.trove.list.TShortList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TShortArrayList;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import gnu.trove.set.TLongSet;
import gnu.trove.set.hash.TLongHashSet;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.deckfour.xes.factory.XFactoryRegistry;
import org.deckfour.xes.model.XEvent;
import org.deckfour.xes.model.XLog;
import org.deckfour.xes.model.XTrace;
import org.deckfour.xes.model.impl.XAttributeLiteralImpl;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.graphbased.directed.petrinet.elements.Arc;
import org.processmining.models.graphbased.directed.petrinet.elements.Place;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;

public class SyncProductFactory {

	public static SyncProduct[] getSyncProduct(Petrinet net, XLog log, XEventClasses classes, TransEvClassMapping map,
			Map<Transition, Integer> mapTrans2Cost, Map<XEventClass, Integer> mapEvClass2Cost,
			Map<Transition, Integer> mapSync2Cost, Marking initialMarking, Marking finalMarking) {

		// build list of move_model transitions
		Transition[] transitions = new Transition[net.getTransitions().size()];
		String[] moveModel = new String[transitions.length];
		TIntList costs = new TIntArrayList();
		Iterator<Transition> it = net.getTransitions().iterator();
		for (int t = 0; t < transitions.length; t++) {
			transitions[t] = it.next();
			moveModel[t] = transitions[t].getLabel() + "_" + t + ",-";
			costs.add(mapTrans2Cost.get(transitions[t]));

		}
		List<String> moves = new ArrayList<String>();

		Place[] placeList = new Place[net.getPlaces().size()];
		String[] placeLabels = new String[net.getPlaces().size()];
		TObjectIntMap<Place> place2index = new TObjectIntHashMap<>(net.getPlaces().size());
		Iterator<Place> itp = net.getPlaces().iterator();
		for (int pi = 0; pi < placeList.length; pi++) {
			placeList[pi] = itp.next();
			place2index.put(placeList[pi], pi);
			placeLabels[pi] = placeList[pi].getLabel();// + "_" + pi;
		}
		List<String> places = new ArrayList<String>();
		TShortList eventNumbers = new TShortArrayList();

		SyncProduct[] result = new SyncProduct[log.size() + 1];
		XTrace trace;
		int startAt = 5;//-1;
		int endAt = 6;//log.size();
		for (int tr = startAt; tr < endAt && tr < log.size(); tr++) {
			System.out.print("Adding trace: ");
			if (tr == -1) {
				trace = XFactoryRegistry.instance().currentDefault().createTrace();
				trace.getAttributes().put("concept:name", new XAttributeLiteralImpl("concept:name", "Empty trace"));
				System.out.println("Empty trace");
			} else {
				trace = log.get(tr);
				System.out.println(trace.getAttributes().get("concept:name"));
			}
			eventNumbers.clear();
			moves.clear();
			moves.addAll(Arrays.asList(moveModel));
			for (int i = 0; i < moves.size(); i++) {
				eventNumbers.add(SyncProduct.NOEVENT);
			}
			places.clear();
			places.addAll(Arrays.asList(placeLabels));

			if (costs.size() > transitions.length) {
				costs.remove(transitions.length, costs.size() - transitions.length);
			}

			XEvent[] events = (XEvent[]) trace.toArray(new XEvent[0]);

			// start with places places
			for (int e = 0; e <= events.length; e++) {
				places.add("pe" + e);
			}

			TIntObjectMap<TLongSet> trans2events = new TIntObjectHashMap<>();

			int sm = transitions.length;
			for (short e = 0; e < events.length; e++) {
				XEventClass clazz = classes.getClassOf(events[e]);
				moves.add("-," + clazz + "_" + e);
				eventNumbers.add(e);
				costs.add(mapEvClass2Cost.get(clazz));
				for (int t = 0; t < transitions.length; t++) {
					if (map.containsKey(transitions[t]) && map.get(transitions[t]).equals(clazz)) {
						// sync move
						moves.add(sm, transitions[t].getLabel() + "_" + t + "," + clazz + "_" + e);
						eventNumbers.insert(sm, e);
						costs.insert(sm,
								mapSync2Cost.get(transitions[t]) == null ? 0 : mapSync2Cost.get(transitions[t]));
						if (trans2events.get(t) == null) {
							trans2events.put(t, new TLongHashSet(3));
						}
						long x = sm;
						x = x << 32;
						x |= e;
						trans2events.get(t).add(x);
						sm++;
					}
				}
			}
			// All moves have been established.

			SyncProductImpl product = new SyncProductImpl(net.getLabel() + " x "
					+ trace.getAttributes().get("concept:name"), moves.toArray(new String[0]),
					places.toArray(new String[0]), eventNumbers.toArray(), costs.toArray());

			for (short t = 0; t < transitions.length; t++) {
				if (trans2events.get(t) != null) {
					for (long sm_e : trans2events.get(t).toArray()) {
						int smt = (int) ((sm_e >>> 32) & 0xFFFFFFFF);
						int e = (int) (sm_e & 0xFFFFFFFF);
						// there's a synchronous product of t with event e which is transition sm
						product.addToOutput(smt, (short) (placeLabels.length + e + 1));
						product.addToInput(smt, (short) (placeLabels.length + e));
					}
				}
				for (short p = 0; p < placeList.length; p++) {
					Arc arc = net.getArc(transitions[t], placeList[p]);
					if (arc != null) {
						int i = 0;
						do {
							// add arc for output in modelmove
							product.addToOutput(t, p);
							if (trans2events.get(t) != null) {
								for (long sm_e : trans2events.get(t).toArray()) {
									int smt = (int) ((sm_e >>> 32) & 0xFFFFFFFF);
									// there's a synchronous product of t with event e which is transition sm
									product.addToOutput(smt, p);
								}
							}
						} while (++i < arc.getWeight());
					}
					arc = net.getArc(placeList[p], transitions[t]);
					if (arc != null) {
						int i = 0;
						do {
							// add arc for output in modelmove
							product.addToInput(t, p);
							if (trans2events.get(t) != null) {
								for (long sm_e : trans2events.get(t).toArray()) {
									int smt = (int) (sm_e >>> 32) & 0xFFFFFFFF;
									int e = (int) (sm_e & 0xFFFFFFFF);
									// there's a synchronous product of t with event e which is transition sm
									product.addToInput(smt, p);
								}
							}
						} while (++i < arc.getWeight());
					}
				}
			}
			for (int e = 0; e < events.length; e++) {
				product.addToInput(sm + e, (short) (placeLabels.length + e));
				product.addToOutput(sm + e, (short) (placeLabels.length + e + 1));
			}

			for (Place place : initialMarking) {
				int p = place2index.get(place);
				product.addToInitialMarking(p);
			}
			product.addToInitialMarking(placeLabels.length);
			for (Place place : finalMarking) {
				int p = place2index.get(place);
				product.addToFinalMarking(p);
			}
			product.addToFinalMarking(placeLabels.length + events.length);
			result[tr - startAt] = product;
		}

		return result;
	}
}
