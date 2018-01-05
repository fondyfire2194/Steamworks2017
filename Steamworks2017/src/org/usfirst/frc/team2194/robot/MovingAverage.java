package org.usfirst.frc.team2194.robot;

public class MovingAverage {
	private int size;
	private double total = 0d;
	private int index = 0;
	private double samples[];

	public MovingAverage(int size) {
		this.size = size;
		samples = new double[size];
		for (int i = 0; i < size; i++)
			samples[i] = 0d;
	}

	public void add(double x) {
		total -= samples[index];
		samples[index] = x;
		total += x;
		if (++index == size) {
			index = 0;
		}
	}

	public double getAverage() {
		return total / size;
	}
}
