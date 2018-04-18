package nl.tue.alignment.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FilenameFilter;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutionException;

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
import org.processmining.plugins.replayer.replayresult.SyncReplayResult;

import nl.tue.alignment.Progress;
import nl.tue.alignment.Replayer;
import nl.tue.alignment.ReplayerParameters;
import nl.tue.alignment.algorithms.ReplayAlgorithm;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

public class AlignmentTest {

	private static final String FOLDER = "c:/temp/alignment/";
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

		mainFileFolder(Debug.STATS, "bpi12");//"pr1151_l4_noise","pr1912_l4_noise");
		//mainFileFolder(Debug.DOT, "test");//"pr1151_l4_noise","pr1912_l4_noise");
		//		mainFileFolder(Debug.STATS, "pr1151_l4_noise", "pr1912_l4_noise", "temp", "sepsis", "prCm6", "prDm6", "prEm6", "prFm6",
		//				"prGm6", "prAm6", "prBm6");
		//		mainFolder(Debug.NONE,"laura/", "isbpm2013/");
	}

	public static void mainFolder(Debug debug, String... eval) throws Exception {

		for (String folder : eval) {

			String[] names = new File(FOLDER + folder).list(new FilenameFilter() {

				public boolean accept(File dir, String name) {
					return name.endsWith(".pnml");
				}
			});

			System.out.println("file,algorithm,traces,time (ms),memory (kb),timeout,cost");
			for (String name : names) {
				name = name.replace(".pnml", "");

				PetrinetGraph net = constructNet(FOLDER + folder + name + ".pnml");
				Marking initialMarking = getInitialMarking(net);
				Marking finalMarking = getFinalMarking(net);
				XLog log;
				XEventClassifier eventClassifier;

				XMxmlParser parser = new XMxmlParser();
				eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
				log = parser.parse(new File(FOLDER + folder + name + ".xml")).get(0);

				doReplayExperiment(debug, FOLDER + folder + name, net, initialMarking, finalMarking, log,
						eventClassifier);

			}
		}
	}

	public static void mainFileFolder(Debug debug, String... names) throws Exception {

		for (String name : names) {
			PetrinetGraph net = constructNet(FOLDER + name + "/" + name + ".pnml");
			Marking initialMarking = getInitialMarking(net);
			Marking finalMarking = getFinalMarking(net);
			XLog log;
			XEventClassifier eventClassifier;

			if (new File(FOLDER + name + "/" + name + ".mxml").exists()) {
				XMxmlParser parser = new XMxmlParser();
				eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
				log = parser.parse(new File(FOLDER + name + "/" + name + ".mxml")).get(0);
			} else {
				XesXmlParser parser = new XesXmlParser();
				eventClassifier = new XEventNameClassifier();
				log = parser.parse(new File(FOLDER + name + "/" + name + ".xes")).get(0);
			}

			doReplayExperiment(debug, FOLDER + name + "/" + name, net, initialMarking, finalMarking, log,
					eventClassifier);

		}
	}

	private static void doReplayExperiment(Debug debug, String folder, PetrinetGraph net, Marking initialMarking,
			Marking finalMarking, XLog log, XEventClassifier eventClassifier)
			throws FileNotFoundException, InterruptedException, ExecutionException {
		XEventClass dummyEvClass = new XEventClass("DUMMY", 99999);
		TransEvClassMapping mapping = constructMapping(net, log, dummyEvClass, eventClassifier);
		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);
		XEventClasses classes = summary.getEventClasses();

		int threads;
		if (debug == Debug.STATS) {
			System.out.println("Started: " + folder);
			threads = Math.max(1, Runtime.getRuntime().availableProcessors() / 2);
		} else if (debug == Debug.DOT) {
			threads = 1;
		} else {
			threads = Math.max(1, Runtime.getRuntime().availableProcessors() / 2);
		}
		//		threads = 1;

		// timeout 30 sec per trace minutes
		int timeout = log.size() * 30 * 1000 / 10;
		int maxNumberOfStates = Integer.MAX_VALUE;
		short[] initSplits = new short[] { 1 };

		boolean moveSort = false;
		boolean useInt = false;
		boolean partialOrder = false;
		boolean preferExact = true;
		boolean queueSort = true;
		ReplayerParameters parameters;

		parameters = new ReplayerParameters.AStarWithMarkingSplit(moveSort, threads, useInt, debug, timeout,
				maxNumberOfStates, partialOrder, initSplits);
		doReplay(debug, folder, "Incre", net, initialMarking, finalMarking, log, mapping, classes, parameters);

		parameters = new ReplayerParameters.AStar(moveSort, queueSort, preferExact, threads, useInt, debug, timeout,
				maxNumberOfStates, partialOrder);
		doReplay(debug, folder, "AStar", net, initialMarking, finalMarking, log, mapping, classes, parameters);
	}

	private static void doReplay(Debug debug, String folder, String postfix, PetrinetGraph net, Marking initialMarking,
			Marking finalMarking, XLog log, TransEvClassMapping mapping, XEventClasses classes,
			ReplayerParameters parameters) throws FileNotFoundException, InterruptedException, ExecutionException {
		PrintStream stream;
		if (debug == Debug.STATS) {
			stream = new PrintStream(new File(folder + " " + postfix + ".csv"));
		} else if (debug == Debug.DOT) {
			stream = new PrintStream(new File(folder + " " + postfix + ".dot"));
		} else {
			stream = System.out;
		}
		ReplayAlgorithm.Debug.setOutputStream(stream);

		Replayer replayer = new Replayer(parameters, (Petrinet) net, initialMarking, finalMarking, log, classes,
				mapping);
		PNRepResult result = replayer.computePNRepResult(Progress.INVISIBLE);

		if (stream != System.out) {
			System.out.println(result.getInfo().toString());
			System.out.println("Completed: " + folder + postfix);
			stream.close();
		} else {

			System.out.print(folder + ",");
			System.out.print(postfix + ",");
			System.out.print((log.size() + 1) + ",");

			int cost = (int) Double.parseDouble((String) result.getInfo().get(Replayer.MAXMODELMOVECOST));
			int timeout = 0;
			int time = 0;
			int mem = 0;
			for (SyncReplayResult res : result) {
				cost += res.getInfo().get(PNRepResult.RAWFITNESSCOST);
				timeout += res.getInfo().get(Replayer.TRACEEXITCODE).intValue() != 1 ? 1 : 0;
				time += res.getInfo().get(PNRepResult.TIME).intValue();
				mem = Math.max(mem, res.getInfo().get(Replayer.MEMORYUSED).intValue());
			}
			System.out.print(time + ",");
			System.out.print(mem + ",");
			System.out.print(timeout + ",");
			System.out.print(cost + ",");

			System.out.println();
			System.out.flush();
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
			if (t.isSilent() || t.getLabel().startsWith("tau") || t.getLabel().equals("t2") || t.getLabel().equals("t8")
					|| t.getLabel().equals("complete")) {
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
