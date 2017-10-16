package nl.tue.alignment;

public interface Progress {

	public final static Progress INVISIBLE = new Progress() {

		public void setMaximum(int maximum) {
		}

		public void inc() {
		}

		public boolean isCancelled() {
			return false;
		}

		public void log(String message) {
			System.out.println(message);
		}

	};

	public void setMaximum(int maximum);

	public void inc();

	public boolean isCancelled();

	public void log(String message);
}
