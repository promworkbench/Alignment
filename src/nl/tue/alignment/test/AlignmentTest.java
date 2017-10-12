package nl.tue.alignment.test;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import nl.tue.alignment.Replayer;
import nl.tue.alignment.ReplayerParameters;
import nl.tue.alignment.algorithms.ReplayAlgorithm;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;
import nl.tue.alignment.algorithms.datastructures.SyncProduct;
import nl.tue.astar.util.ilp.LPMatrixException;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClassifier;
import org.deckfour.xes.in.XMxmlParser;
import org.deckfour.xes.info.XLogInfo;
import org.deckfour.xes.info.XLogInfoFactory;
import org.deckfour.xes.info.impl.XLogInfoImpl;
import org.deckfour.xes.model.XLog;
import org.jbpt.petri.Flow;
import org.jbpt.petri.NetSystem;
import org.jbpt.petri.io.PNMLSerializer;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.graphbased.directed.petrinet.PetrinetGraph;
import org.processmining.models.graphbased.directed.petrinet.elements.Place;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.models.graphbased.directed.petrinet.impl.PetrinetFactory;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.astar.petrinet.impl.AbstractPILPDelegate;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;
import org.processmining.plugins.petrinet.replayresult.PNRepResult;

public class AlignmentTest {

	public static int iteration = 0;

	static {
		try {
			System.loadLibrary("lpsolve55");
			System.loadLibrary("lpsolve55j");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public static void main(String[] args) throws Exception {
		test(args);
	}

	public static void test(String[] args) throws Exception {
		//		DummyUIPluginContext context = new DummyUIPluginContext(new DummyGlobalContext(), "label");
		//		AbstractPILPDelegate.setDebugMode(new File("D:\\temp\\alignmentDebugTest\\"));

		AbstractPILPDelegate.setDebugMode(null);

		PetrinetGraph net = null;
		Marking initialMarking = null;
		Marking finalMarking = null; // only one marking is used so far
		XLog log = null;
		Map<Transition, Integer> costMOS = null; // movements on system
		Map<XEventClass, Integer> costMOT = null; // movements on trace
		TransEvClassMapping mapping = null;

		String name = "prAm6";
		net = constructNet("d:/temp/alignment/" + name + "/" + name + ".pnml");
		initialMarking = getInitialMarking(net);
		finalMarking = getFinalMarking(net);
		//		log = XParserRegistry.instance().currentDefault().parse(new File("d:/temp/alignment/prAm6.mxml"))
		//				.get(0);
		XMxmlParser parser = new XMxmlParser();
		log = parser.parse(new File("d:/temp/alignment/" + name + "/" + name + ".mxml")).get(0);

		//		log.retainAll(Arrays.asList(new XTrace[] { log.get(201) }));

		//		log.add(XFactoryRegistry.instance().currentDefault().createTrace());

		//			log = XParserRegistry.instance().currentDefault().parse(new File("d:/temp/BPI 730858110.xes.gz")).get(0);
		//			log = XFactoryRegistry.instance().currentDefault().openLog();
		XEventClass dummyEvClass = new XEventClass("DUMMY", 99999);
		XEventClassifier eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
		mapping = constructMapping(net, log, dummyEvClass, eventClassifier);

		//		doTestOldAlignments(net, initialMarking, finalMarkings, log, costMOS, costMOT, mapping);
		doTestNewAlignments((Petrinet) net, initialMarking, finalMarking, log, mapping);
	}

	protected static void doTestNewAlignments(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log,
			TransEvClassMapping mapping) {

		ReplayAlgorithm.Debug.setOutputStream(System.out);
		ReplayerParameters parameters = new ReplayerParameters.AStarWithMarkingSplit(true, false, Debug.STATS);
		Replayer replayer = new Replayer(parameters, net, initialMarking, finalMarking, log, mapping);
		try {
			PNRepResult result = replayer.computePNRepResult();
			System.out.println(result.toString());
		} catch (LPMatrixException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

	}

	private static PetrinetGraph constructNet(String netFile) {
		PNMLSerializer PNML = new PNMLSerializer();
		NetSystem sys = PNML.parse(netFile);

		//System.err.println(sys.getMarkedPlaces());

		//		int pi, ti;
		//		pi = ti = 1;
		//		for (org.jbpt.petri.Place p : sys.getPlaces())
		//			p.setName("p" + pi++);
		//		for (org.jbpt.petri.Transition t : sys.getTransitions())
		//				t.setName("t" + ti++);

		PetrinetGraph net = PetrinetFactory.newPetrinet(netFile);

		// places
		Map<org.jbpt.petri.Place, Place> p2p = new HashMap<org.jbpt.petri.Place, Place>();
		for (org.jbpt.petri.Place p : sys.getPlaces()) {
			Place pp = net.addPlace(p.toString());
			p2p.put(p, pp);
		}

		// transitions
		Map<org.jbpt.petri.Transition, Transition> t2t = new HashMap<org.jbpt.petri.Transition, Transition>();
		for (org.jbpt.petri.Transition t : sys.getTransitions()) {
			Transition tt = net.addTransition(t.getLabel());
			tt.setInvisible(t.isSilent());
			t2t.put(t, tt);
		}

		// flow
		for (Flow f : sys.getFlow()) {
			if (f.getSource() instanceof org.jbpt.petri.Place) {
				net.addArc(p2p.get(f.getSource()), t2t.get(f.getTarget()));
			} else {
				net.addArc(t2t.get(f.getSource()), p2p.get(f.getTarget()));
			}
		}

		// add unique start node
		if (sys.getSourceNodes().isEmpty()) {
			Place i = net.addPlace("START_P");
			Transition t = net.addTransition("");
			t.setInvisible(true);
			net.addArc(i, t);

			for (org.jbpt.petri.Place p : sys.getMarkedPlaces()) {
				net.addArc(t, p2p.get(p));
			}

		}

		return net;
	}

	private static Marking getFinalMarking(PetrinetGraph net) {
		Marking finalMarking = new Marking();

		for (Place p : net.getPlaces()) {
			if (net.getOutEdges(p).isEmpty())
				finalMarking.add(p);
		}

		return finalMarking;
	}

	private static Marking getInitialMarking(PetrinetGraph net) {
		Marking initMarking = new Marking();

		for (Place p : net.getPlaces()) {
			if (net.getInEdges(p).isEmpty())
				initMarking.add(p);
		}

		return initMarking;
	}

	private static TransEvClassMapping constructMapping(PetrinetGraph net, XLog log, XEventClass dummyEvClass,
			XEventClassifier eventClassifier) {
		TransEvClassMapping mapping = new TransEvClassMapping(eventClassifier, dummyEvClass);

		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);

		for (Transition t : net.getTransitions()) {
			for (XEventClass evClass : summary.getEventClasses().getClasses()) {
				String id = evClass.getId();

				if (t.getLabel().equals(id)) {
					mapping.put(t, evClass);
					break;
				}
			}

			//			if (!mapped && !t.isInvisible()) {
			//				mapping.put(t, dummyEvClass);
			//			}

		}

		return mapping;
	}

	public static int logMoveCost(SyncProduct product, short[] alignment) {
		return getCostForType(product, alignment, SyncProduct.LOG_MOVE, SyncProduct.LOG_MOVE);
	}

	public static int modelMoveCost(SyncProduct product, short[] alignment) {
		return getCostForType(product, alignment, SyncProduct.MODEL_MOVE, SyncProduct.TAU_MOVE);
	}

	public static int syncMoveCost(SyncProduct product, short[] alignment) {
		return getCostForType(product, alignment, SyncProduct.SYNC_MOVE, SyncProduct.SYNC_MOVE);
	}

	private static int getCostForType(SyncProduct product, short[] alignment, byte type1, byte type2) {
		int cost = 0;
		for (int i = 0; i < alignment.length; i++) {
			if (product.getTypeOf(alignment[i]) == type1 || product.getTypeOf(alignment[i]) == type2) {
				cost += product.getCost(alignment[i]);
			}
		}
		return cost;
	}

}
