package nl.tue.alignment.test;

import java.io.File;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Map;

import nl.tue.alignment.Progress;
import nl.tue.alignment.Replayer;
import nl.tue.alignment.ReplayerParameters;
import nl.tue.alignment.algorithms.ReplayAlgorithm;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.deckfour.xes.classification.XEventClassifier;
import org.deckfour.xes.classification.XEventNameClassifier;
import org.deckfour.xes.in.XMxmlParser;
import org.deckfour.xes.in.XesXmlParser;
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

		Debug debug = Debug.STATS;

		String[] names = new String[] { "sepsis" };//, "prCm6", "prDm6", "prEm6", "prFm6", "prGm6",  "prAm6", "prBm6" };
		for (String name : names) {

			PetrinetGraph net = constructNet("d:/temp/alignment/" + name + "/" + name + ".pnml");
			Marking initialMarking = getInitialMarking(net);
			Marking finalMarking = getFinalMarking(net);
			XLog log;
			XEventClassifier eventClassifier;

			if (new File("d:/temp/alignment/" + name + "/" + name + ".mxml").exists()) {
				XMxmlParser parser = new XMxmlParser();
				eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
				log = parser.parse(new File("d:/temp/alignment/" + name + "/" + name + ".mxml")).get(0);
			} else {
				XesXmlParser parser = new XesXmlParser();
				eventClassifier = new XEventNameClassifier();
				log = parser.parse(new File("d:/temp/alignment/" + name + "/" + name + ".xes")).get(0);
			}
			XEventClass dummyEvClass = new XEventClass("DUMMY", 99999);
			TransEvClassMapping mapping = constructMapping(net, log, dummyEvClass, eventClassifier);
			XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);
			XEventClasses classes = summary.getEventClasses();

			System.out.println("Started: " + name);

			PrintStream stream;
			if (debug == Debug.STATS) {
				stream = new PrintStream(new File("d:/temp/alignment/" + name + "/" + name + ".csv"));
			} else if (debug == Debug.DOT) {
				stream = new PrintStream(new File("d:/temp/alignment/" + name + "/" + name + ".dot"));
			} else {
				stream = System.out;
			}
			ReplayAlgorithm.Debug.setOutputStream(stream);

			// timeout 60 minutes
			int timeout = 60 * 60 * 1000;
			int initBins = 1;

			ReplayerParameters parameters = new ReplayerParameters.AStarWithMarkingSplit(false, Math.max(1, Runtime
					.getRuntime().availableProcessors() / 2), false, initBins, debug, timeout, true);

			//			ReplayerParameters parameters = new ReplayerParameters.AStar(false, true, true, Math.max(1, Runtime
			//					.getRuntime().availableProcessors() / 2), false, debug, timeout);

			Replayer replayer = new Replayer(parameters, (Petrinet) net, initialMarking, finalMarking, log, classes,
					mapping);
			PNRepResult result = replayer.computePNRepResult(Progress.INVISIBLE);

			System.out.println(result.getInfo().toString());
			System.out.println("Completed: " + name);

			stream.close();
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
			if (t.isSilent() || t.getLabel().startsWith("tau") || t.getLabel().equals("t2")
					|| t.getLabel().equals("t8") || t.getLabel().equals("complete")) {
				tt.setInvisible(true);
			}
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

}
