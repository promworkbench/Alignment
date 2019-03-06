package nl.tue.alignment.algorithms.syncproduct.petrinet;

import java.util.Arrays;

class TransitionEventClassList {
	public static final int NOMOVE = Integer.MIN_VALUE;

	public static final TransitionEventClassList EMPTY = new TransitionEventClassList();

	private final int[] transitions;
	private final int[] eventClasses;

	private TransitionEventClassList() {
		transitions = new int[] {};
		eventClasses = new int[] {};
	}

	public TransitionEventClassList(int transition) {
		transitions = new int[] { transition };
		eventClasses = new int[] { NOMOVE };
	}

	public TransitionEventClassList(int transition, int eventClass) {
		transitions = new int[] { transition };
		eventClasses = new int[] { eventClass };
	}

	public TransitionEventClassList(TransitionEventClassList first, TransitionEventClassList second) {
		transitions = new int[first.transitions.length + second.transitions.length];
		eventClasses = new int[first.eventClasses.length + second.eventClasses.length];
		System.arraycopy(first.transitions, 0, transitions, 0, first.transitions.length);
		System.arraycopy(second.transitions, 0, transitions, first.transitions.length, second.transitions.length);
		System.arraycopy(first.eventClasses, 0, eventClasses, 0, first.eventClasses.length);
		System.arraycopy(second.eventClasses, 0, eventClasses, first.eventClasses.length, second.eventClasses.length);
	}

	public int hashCode() {
		return Arrays.hashCode(eventClasses);
	}

	public boolean equals(Object o) {
		return (o instanceof TransitionEventClassList
				? Arrays.equals(eventClasses, ((TransitionEventClassList) o).eventClasses)
				: false);
	}

	public String toString() {
		StringBuilder b = new StringBuilder();
		b.append('[');
		for (int i = 0; i < eventClasses.length; i++) {
			if (eventClasses[i] != NOMOVE) {
				b.append(eventClasses[i]);
				b.append(", ");
			}
		}
		return b.append(']').toString();
	}

}