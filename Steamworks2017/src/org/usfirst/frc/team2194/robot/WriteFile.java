package org.usfirst.frc.team2194.robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class WriteFile {

	public WriteFile(String filename, int[] x, int[] y, double[] z) throws IOException {
		BufferedWriter outputWriter = null;
		outputWriter = new BufferedWriter(new FileWriter(filename));

		outputWriter.write("Heights ");
		outputWriter.write(Integer.toString(x.length));
		outputWriter.write(" :- ");

		for (int i = 0; i < x.length; i++) {
			outputWriter.write(Integer.toString(x[i]));
			outputWriter.write(',');
		}
		outputWriter.newLine();

		outputWriter.write("Widths    ");
		outputWriter.write(Integer.toString(y.length));
		outputWriter.write(" :- ");

		for (int i = 0; i < y.length; i++) {
			outputWriter.write(Integer.toString(y[i]));
			outputWriter.write(',');
		}
		outputWriter.newLine();

		outputWriter.write("Distances    ");
		outputWriter.write(Integer.toString(z.length));
		outputWriter.write(" :- ");

		for (int i = 0; i < z.length; i++) {

			outputWriter.write(Double.toString(z[i]));
			outputWriter.write(',');
		}
		outputWriter.newLine();

		outputWriter.flush();
		outputWriter.close();
	}
}
