package nl.tue.alignment.algorithms.syncproduct;

import java.util.Arrays;

class StringList {
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
		Arrays.fill(list, size, list.length, null);
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