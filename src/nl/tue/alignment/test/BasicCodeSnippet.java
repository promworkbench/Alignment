package nl.tue.alignment.test;

import java.util.concurrent.Future;

import org.deckfour.xes.classification.XEventClasses;
import org.deckfour.xes.model.XLog;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;
import org.processmining.plugins.replayer.replayresult.SyncReplayResult;

import nl.tue.alignment.Replayer;
import nl.tue.alignment.TraceByTraceAlignment;
import nl.tue.alignment.TraceReplayTask;
import nl.tue.alignment.TraceReplayTask.TraceReplayResult;
import nl.tue.alignment.Utils;

public class BasicCodeSnippet {

	public void doReplay(XLog log, Petrinet net, Marking initialMarking, Marking finalMarking, XEventClasses classes,
			TransEvClassMapping mapping) {

		TraceByTraceAlignment alignmentCalculator = new TraceByTraceAlignment(net, initialMarking, finalMarking,
				classes, mapping);

		// timeout per trace in milliseconds
		int timeoutMilliseconds = 10 * 1000;
		// preprocessing time to be added to the statistics if necessary
		long preProcessTimeNanoseconds = 0;

		int fitting = 0;
		int nonfitting = 0;

		for (int i = 0; i < log.size(); i++) {
			try {
				// synchronously execute the alignment for this trace.
				Future<TraceReplayTask> task = alignmentCalculator.doReplay(log.get(i), i, timeoutMilliseconds,
						preProcessTimeNanoseconds);
				// get the result
				TraceReplayResult result = task.get().getResult();
				switch (result) {
					case DUPLICATE :
						assert false; // cannot happen in this setting
						throw new RuntimeException("Result cannot be a duplicate in per-trace computations.");
					case FAILED :
						// internal error in the construction of synchronous product or other error.
						throw new RuntimeException("Error in alignment computations");
					case SUCCESS :
						// process succcesful execution of the replayer
						SyncReplayResult replayResult = task.get().getSuccesfulResult();
						int exitCode = replayResult.getInfo().get(Replayer.TRACEEXITCODE).intValue();
						if ((exitCode & Utils.OPTIMALALIGNMENT) == Utils.OPTIMALALIGNMENT) {
							// Optimal alignment found.
							fitting++;
						} else if ((exitCode & Utils.FAILEDALIGNMENT) == Utils.FAILEDALIGNMENT) {
							// failure in the alignment. Error code shows more details.
							nonfitting++;
						}
						if ((exitCode & Utils.ENABLINGBLOCKEDBYOUTPUT) == Utils.ENABLINGBLOCKEDBYOUTPUT) {
							// in some marking, there were too many tokens in a place, blocking the addition of more tokens. Current upper limit is 128
						}
						if ((exitCode & Utils.COSTFUNCTIONOVERFLOW) == Utils.COSTFUNCTIONOVERFLOW) {
							// in some marking, the cost function went through the upper limit of 2^24
						}
						if ((exitCode & Utils.HEURISTICFUNCTIONOVERFLOW) == Utils.HEURISTICFUNCTIONOVERFLOW) {
							// in some marking, the heuristic function went through the upper limit of 2^24
						}
						if ((exitCode & Utils.TIMEOUTREACHED) == Utils.TIMEOUTREACHED) {
							// alignment failed with a timeout
						}
						if ((exitCode & Utils.STATELIMITREACHED) == Utils.STATELIMITREACHED) {
							// alignment failed due to reacing too many states.
						}
						if ((exitCode & Utils.COSTLIMITREACHED) == Utils.COSTLIMITREACHED) {
							// no optimal alignment found with cost less or equal to the given limit.
						}

						break;
				}

			} catch (Exception e) {
				e.printStackTrace();
			}

		}
	}
}
