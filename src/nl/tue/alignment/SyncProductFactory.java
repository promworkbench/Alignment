package nl.tue.alignment;

import gnu.trove.iterator.TShortIterator;
import gnu.trove.list.TIntList;
import gnu.trove.list.TShortList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TShortArrayList;
import gnu.trove.map.TObjectShortMap;
import gnu.trove.map.TShortObjectMap;
import gnu.trove.map.hash.TObjectShortHashMap;
import gnu.trove.map.hash.TShortObjectHashMap;
import gnu.trove.set.TShortSet;
import gnu.trove.set.hash.TShortHashSet;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.deckfour.xes.extension.std.XConceptExtension;
import org.deckfour.xes.model.XTrace;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.graphbased.directed.petrinet.PetrinetEdge;
import org.processmining.models.graphbased.directed.petrinet.elements.Place;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;

public class SyncProductFactory {

	private static class StringList {
		private String[] list;
		int size = 0;

		public StringList(int capacity) {
			list = new String[capacity];
		}

		public void add(String s) {
			ensureCapacity(size);
			list[size] = s;
			size++;
		}

		private void ensureCapacity(int insertAt) {
			if (list.length <= insertAt) {
				list = Arrays.copyOf(list, list.length * 2);
			}
		}

		public void trunctate(int size) {
			this.size = size;
		}

		public String get(int index) {
			return list[index];
		}

		public int size() {
			return size;
		}

		public String[] asArray() {
			return Arrays.copyOf(list, size);
		}

		public String toString() {

			int iMax = size - 1;
			if (iMax == -1)
				return "[]";

			StringBuilder b = new StringBuilder();
			b.append('[');
			for (int i = 0;; i++) {
				b.append(list[i]);
				if (i == iMax)
					return b.append(']').toString();
				b.append(", ");
			}
		}
	}

	private final int transitions;
	private final TIntList t2mmCost;
	private final TIntList t2smCost;
	private final TObjectShortMap<Object> t2id;
	private final StringList t2name;
	private final List<short[]> t2input;
	private final List<short[]> t2output;
	private final TShortList t2eid;

	private final int classCount;
	private final TObjectShortMap<XEventClass> c2id;
	private final TIntList c2lmCost;
	private final TShortObjectMap<TShortSet> c2t;

	private final int places;
	private final StringList p2name;
	private final byte[] initMarking;
	private final byte[] finMarking;
	private final XEventClasses classes;
	private final String label;

	public SyncProductFactory(Petrinet net, XEventClasses classes, TransEvClassMapping map,
			Map<Transition, Integer> mapTrans2Cost, Map<XEventClass, Integer> mapEvClass2Cost,
			Map<Transition, Integer> mapSync2Cost, Marking initialMarking, Marking finalMarking) {
		label = net.getLabel();
		this.classes = classes;
		this.classCount = classes.size();
		c2id = new TObjectShortHashMap<>(this.classCount, 0.75f, (short) -1);
		c2lmCost = new TIntArrayList(this.classCount);
		c2t = new TShortObjectHashMap<>(this.classCount);
		for (XEventClass clazz : classes.getClasses()) {
			c2id.put(clazz, (short) c2lmCost.size());
			c2lmCost.add(mapEvClass2Cost.get(clazz));
		}

		transitions = net.getTransitions().size();
		t2mmCost = new TIntArrayList(transitions * 2);
		t2smCost = new TIntArrayList(transitions * 2);
		t2eid = new TShortArrayList(transitions * 2);
		t2name = new StringList(transitions * 2);
		t2input = new ArrayList<>(transitions * 2);
		t2output = new ArrayList<>(transitions * 2);

		places = net.getPlaces().size();
		p2name = new StringList(places * 2);
		TObjectShortMap<Place> p2id = new TObjectShortHashMap<>(net.getPlaces().size(), 0.75f, (short) -1);
		t2id = new TObjectShortHashMap<>(net.getTransitions().size(), 0.75f, (short) -1);

		// build list of move_model transitions
		Integer cost;
		Iterator<Transition> it = net.getTransitions().iterator();
		while (it.hasNext()) {
			Transition t = it.next();
			t2id.put(t, (short) t2name.size());

			// update mapping from event class to transitions
			XEventClass clazz = map.get(t);
			if (clazz != null) {
				TShortSet set = c2t.get(c2id.get(clazz));
				if (set == null) {
					set = new TShortHashSet(3);
					c2t.put(c2id.get(clazz), set);
				}
				set.add((short) t2name.size());
			}

			cost = mapTrans2Cost.get(t);
			t2mmCost.add(cost == null ? 0 : cost);
			cost = mapSync2Cost.get(t);
			t2smCost.add(cost == null ? 0 : cost);
			t2name.add(t.getLabel());
			t2eid.add(SyncProduct.NOEVENT);

			short[] input = new short[net.getInEdges(t).size()];
			int i = 0;
			for (PetrinetEdge<?, ?> e : net.getInEdges(t)) {
				Place p = (Place) e.getSource();
				short id = p2id.get(p);
				if (id == -1) {
					id = (short) p2id.size();
					p2id.put(p, id);
					p2name.add(p.getLabel());
				}
				input[i++] = id;
			}
			t2input.add(input);
			short[] output = new short[net.getOutEdges(t).size()];
			i = 0;
			for (PetrinetEdge<?, ?> e : net.getOutEdges(t)) {
				Place p = (Place) e.getTarget();
				short id = p2id.get(p);
				if (id == -1) {
					id = (short) p2id.size();
					p2id.put(p, id);
					p2name.add(p.getLabel());
				}
				output[i++] = id;
			}
			t2output.add(output);
		}

		initMarking = new byte[p2name.size()];
		for (Place p : initialMarking) {
			short id = p2id.get(p);
			if (id >= 0) {
				initMarking[id]++;
			}
		}

		finMarking = new byte[p2name.size()];
		for (Place p : finalMarking) {
			short id = p2id.get(p);
			if (id >= 0) {
				finMarking[id]++;
			}
		}

	}

	public SyncProduct getSyncProduct(XTrace trace) {
		// for this trace, compute the log-moves
		// compute the sync moves
		for (short e = 0; e < trace.size(); e++) {
			XEventClass clazz = classes.getClassOf(trace.get(e));
			short cid = c2id.get(clazz);
			// add a place
			p2name.add("pe_" + e);
			// add log move
			t2name.add(clazz.toString());
			t2mmCost.add(c2lmCost.get(cid));
			t2eid.add(e);

			TShortSet set = c2t.get(cid);
			if (set != null) {
				TShortIterator it = set.iterator();
				while (it.hasNext()) {
					// add sync move
					short t = it.next();
					t2name.add(t2name.get(t) + "," + clazz);
					t2mmCost.add(t2smCost.get(t));
					t2eid.add(e);
				}
			}
		}
		if (trace.size() > 0) {
			p2name.add("pe_" + trace.size());
		}

		String traceLabel = XConceptExtension.instance().extractName(trace);
		if (traceLabel == null) {
			traceLabel = "XTrace@" + Integer.toHexString(trace.hashCode());
		}
		SyncProductImpl product = new SyncProductImpl(label + " x " + traceLabel, //label
				t2name.asArray(), //transition labels
				p2name.asArray(), // place labels
				t2eid.toArray(), //event numbers
				t2mmCost.toArray());

		short t = 0;
		for (; t < transitions; t++) {
			// first the model moves
			product.setInput(t, t2input.get(t));
			product.setOutput(t, t2output.get(t));
		}

		for (short e = 0; e < trace.size(); e++) {
			XEventClass clazz = classes.getClassOf(trace.get(e));
			short cid = c2id.get(clazz);
			product.setInput(t, places + e);
			product.setOutput(t, places + e + 1);
			t++;

			TShortSet set = c2t.get(cid);
			if (set != null) {
				TShortIterator it = set.iterator();
				while (it.hasNext()) {
					// add sync move
					short t2 = it.next();
					product.setInput(t, t2input.get(t2));
					product.setOutput(t, t2output.get(t2));

					product.addToInput(t, (short) (places + e));
					product.addToOutput(t, (short) (places + e + 1));
					t++;
				}
			}
		}

		product.setInitialMarking(Arrays.copyOf(initMarking, p2name.size()));
		product.setFinalMarking(Arrays.copyOf(finMarking, p2name.size()));
		if (trace.size() > 0) {
			product.addToInitialMarking(places);
			product.addToFinalMarking(places + trace.size());
		}

		// trim to size;
		p2name.trunctate(places);
		t2name.trunctate(transitions);
		t2mmCost.remove(transitions, t2mmCost.size() - transitions);
		t2eid.remove(transitions, t2eid.size() - transitions);

		return product;
	}
}
