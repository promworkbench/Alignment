package nl.tue.alignment;

public interface Canceller {

	public final static Canceller DEFAULT = new Canceller() {

		public boolean isCancelled() {
			return false;
		}

	};

	public boolean isCancelled();

}
