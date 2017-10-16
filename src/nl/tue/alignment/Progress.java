package nl.tue.alignment;

public interface Progress {

	public final static Progress INVISIBLE = new Progress() {

		public void setMaximum(int maximum) {
		}

		public void inc() {
		}

	};

	public void setMaximum(int maximum);

	public void inc();
}
