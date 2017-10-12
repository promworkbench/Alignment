package nl.tue.alignment.test;

import gnu.trove.map.TObjectIntMap;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.AStarLargeLP;
import nl.tue.alignment.algorithms.ReplayAlgorithm;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;
import nl.tue.alignment.algorithms.datastructures.SyncProduct;
import nl.tue.alignment.algorithms.datastructures.SyncProductFactory;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.deckfour.xes.classification.XEventClassifier;
import org.deckfour.xes.extension.std.XConceptExtension;
import org.deckfour.xes.in.XMxmlParser;
import org.deckfour.xes.info.XLogInfo;
import org.deckfour.xes.info.XLogInfoFactory;
import org.deckfour.xes.info.impl.XLogInfoImpl;
import org.deckfour.xes.model.XEvent;
import org.deckfour.xes.model.XLog;
import org.deckfour.xes.model.XTrace;
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
import org.processmining.plugins.petrinet.replayresult.PNRepResultImpl;
import org.processmining.plugins.replayer.replayresult.SyncReplayResult;

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
		Marking[] finalMarkings = null; // only one marking is used so far
		XLog log = null;
		Map<Transition, Integer> costMOS = null; // movements on system
		Map<XEventClass, Integer> costMOT = null; // movements on trace
		TransEvClassMapping mapping = null;

		String name = "prFm6";
		net = constructNet("d:/temp/alignment/" + name + "/" + name + ".pnml");
		initialMarking = getInitialMarking(net);
		finalMarkings = getFinalMarkings(net);
		//		log = XParserRegistry.instance().currentDefault().parse(new File("d:/temp/alignment/prAm6.mxml"))
		//				.get(0);
		XMxmlParser parser = new XMxmlParser();
		log = parser.parse(new File("d:/temp/alignment/" + name + "/" + name + ".mxml")).get(0);

		//		log.retainAll(Arrays.asList(new XTrace[] { log.get(201) }));

		//		log.add(XFactoryRegistry.instance().currentDefault().createTrace());

		//			log = XParserRegistry.instance().currentDefault().parse(new File("d:/temp/BPI 730858110.xes.gz")).get(0);
		//			log = XFactoryRegistry.instance().currentDefault().openLog();
		costMOS = constructMOSCostFunction(net);
		XEventClass dummyEvClass = new XEventClass("DUMMY", 99999);
		XEventClassifier eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
		costMOT = constructMOTCostFunction(net, log, eventClassifier, dummyEvClass);
		mapping = constructMapping(net, log, dummyEvClass, eventClassifier);

		//		doTestOldAlignments(net, initialMarking, finalMarkings, log, costMOS, costMOT, mapping);
		doTestNewAlignments((Petrinet) net, initialMarking, finalMarkings, log, costMOS, costMOT, mapping);
	}

	protected static PNRepResult doTestNewAlignments(Petrinet net, Marking initialMarking, Marking[] finalMarkings,
			XLog log, Map<Transition, Integer> costMOS, Map<XEventClass, Integer> costMOT, TransEvClassMapping mapping) {

		XEventClassifier eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);
		XEventClasses classes = summary.getEventClasses();

		SyncProductFactory factory = new SyncProductFactory(net, classes, mapping, costMOS, costMOT,
				new HashMap<Transition, Integer>(1), initialMarking, finalMarkings[0]);

		net = null;
		eventClassifier = null;
		summary = null;
		classes = null;
		System.gc();

		List<SyncReplayResult> result = new ArrayList<>();
		try {
			long cost = 0;
			int splits = 0;
			int memUsed = 0;
			long start = System.nanoTime();
			//			OutputStreamWriter writer = new OutputStreamWriter(System.out);

			SyncProduct product = factory.getSyncProduct();

			//				try {
			//					((SyncProductImpl) product).toTpn(writer);
			//					writer.flush();
			//				} catch (IOException e) {
			//					// TODO Auto-generated catch block
			//					e.printStackTrace();
			//				}

			System.out.print("T:Num");
			System.out.print(",");
			System.out.print("T:Name");
			System.out.print(",");
			System.out.print("T:Length");
			System.out.print(",");
			System.out.print("SP:trans");
			System.out.print(",");
			System.out.print("SP:places");
			System.out.print(",");
			System.out.print("A:exitCode");
			System.out.print(",");
			System.out.print("A:cost");
			System.out.print(",");
			System.out.print("A:time(us)");
			System.out.print(",");
			System.out.print("A:splits");
			System.out.print(",");
			System.out.print("A:LMCost");
			System.out.print(",");
			System.out.print("A:MMCost");
			System.out.print(",");
			System.out.print("A:SMCost");
			System.out.println();

			int t = 0;

			int maxCost = 0;
			if (product != null) {
				//				System.out.println("---------------------------- " + product.getLabel());
				//				System.out.println("Trace: " + t + " / " + log.size());
				//				System.out.println("Transitions: " + product.numTransitions());
				//				System.out.println("Places: " + product.numPlaces());

				ReplayAlgorithm algorithm = new AStarLargeLP(product, false, false, Debug.NONE);
				//				ReplayAlgorithm algorithm = new AStar(product);

				//							true, // moveSort on total order
				//							false, // use Integers
				//							Debug.NORMAL // debug mode
				//					);
				short[] alignment = algorithm.run();
				TObjectIntMap<Statistic> stats = algorithm.getStatistics();
				splits = Math.max(splits, stats.get(Statistic.SPLITS));
				maxCost = stats.get(Statistic.COST);
				cost = stats.get(Statistic.COST);
				memUsed = Math.max(memUsed, stats.get(Statistic.MEMORYUSED));

				System.out.print(t);
				System.out.print(",");
				System.out.print("Empty");
				System.out.print(",");
				System.out.print("0");
				System.out.print(",");
				System.out.print(product.numTransitions());
				System.out.print(",");
				System.out.print(product.numPlaces());
				System.out.print(",");
				System.out.print(stats.get(Statistic.EXITCODE));
				System.out.print(",");
				System.out.print(stats.get(Statistic.COST));
				System.out.print(",");
				System.out.print(stats.get(Statistic.TOTALTIME));
				System.out.print(",");
				System.out.print(stats.get(Statistic.SPLITS));
				System.out.print(",");
				System.out.print(logMoveCost(product, alignment));
				System.out.print(",");
				System.out.print(modelMoveCost(product, alignment));
				System.out.print(",");
				System.out.print(syncMoveCost(product, alignment));
				System.out.println();

			}

			for (XTrace trace : log) {
				t++;
				product = factory.getSyncProduct(trace);

				if (product != null) {
					//					System.out.println("---------------------------- " + product.getLabel());
					//					System.out.println("Trace: " + t + " / " + log.size());
					//					System.out.println("Transitions: " + product.numTransitions());
					//					System.out.println("Places: " + product.numPlaces());

					ReplayAlgorithm algorithm = new AStarLargeLP(product, false, false, Debug.NONE);

					short[] alignment = algorithm.run();
					TObjectIntMap<Statistic> stats = algorithm.getStatistics();
					int traceCost = getTraceCost(trace, classes, costMOT);
					SyncReplayResult srr = Utils.toSyncReplayResult(factory, stats, alignment, trace, t - 1);
					srr.addInfo(PNRepResult.TRACEFITNESS,
							1 - (srr.getInfo().get(PNRepResult.RAWFITNESSCOST) / (maxCost + traceCost)));

					result.add(srr);

					splits = Math.max(splits, stats.get(Statistic.SPLITS));
					cost += stats.get(Statistic.COST);
					memUsed = Math.max(memUsed, stats.get(Statistic.MEMORYUSED));

					System.out.print(t);
					System.out.print(",");
					System.out.print(XConceptExtension.instance().extractName(trace));
					System.out.print(",");
					System.out.print(trace.size());
					System.out.print(",");
					System.out.print(product.numTransitions());
					System.out.print(",");
					System.out.print(product.numPlaces());
					System.out.print(",");
					System.out.print(stats.get(Statistic.EXITCODE));
					System.out.print(",");
					System.out.print(stats.get(Statistic.COST));
					System.out.print(",");
					System.out.print(stats.get(Statistic.TOTALTIME));
					System.out.print(",");
					System.out.print(stats.get(Statistic.SPLITS));
					System.out.print(",");
					System.out.print(logMoveCost(product, alignment));
					System.out.print(",");
					System.out.print(modelMoveCost(product, alignment));
					System.out.print(",");
					System.out.print(syncMoveCost(product, alignment));
					System.out.println();

				}

			}
			System.out.println("Total time:     " + (System.nanoTime() - start) / 1000000.0 + " ms");
			System.out.println("Total cost:     " + cost);
			System.out.println("Maximum splits: " + splits);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return new PNRepResultImpl(result);
	}

	private static int getTraceCost(XTrace trace, XEventClasses classes, Map<XEventClass, Integer> costMOT) {
		int cost = 0;
		for (XEvent e : trace) {
			cost += costMOT.get(classes.getClassOf(e));
		}
		return cost;
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

	private static Marking[] getFinalMarkings(PetrinetGraph net) {
		Marking finalMarking = new Marking();

		for (Place p : net.getPlaces()) {
			if (net.getOutEdges(p).isEmpty())
				finalMarking.add(p);
		}

		Marking[] finalMarkings = new Marking[1];
		finalMarkings[0] = finalMarking;

		return finalMarkings;
	}

	private static Marking getInitialMarking(PetrinetGraph net) {
		Marking initMarking = new Marking();

		for (Place p : net.getPlaces()) {
			if (net.getInEdges(p).isEmpty())
				initMarking.add(p);
		}

		return initMarking;
	}

	private static Map<Transition, Integer> constructMOSCostFunction(PetrinetGraph net) {
		Map<Transition, Integer> costMOS = new HashMap<Transition, Integer>();

		for (Transition t : net.getTransitions())
			if (t.isInvisible() || t.getLabel().equals(""))
				costMOS.put(t, 0);
			else
				costMOS.put(t, 1);

		return costMOS;
	}

	private static Map<XEventClass, Integer> constructMOTCostFunction(PetrinetGraph net, XLog log,
			XEventClassifier eventClassifier, XEventClass dummyEvClass) {
		Map<XEventClass, Integer> costMOT = new HashMap<XEventClass, Integer>();
		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);

		for (XEventClass evClass : summary.getEventClasses().getClasses()) {
			costMOT.put(evClass, 1);
		}

		//		costMOT.put(dummyEvClass, 1);

		return costMOT;
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
