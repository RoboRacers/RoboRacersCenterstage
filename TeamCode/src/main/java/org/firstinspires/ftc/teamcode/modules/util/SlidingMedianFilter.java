package org.firstinspires.ftc.teamcode.modules.util;
import java.util.Arrays;

public class SlidingMedianFilter {
    private double[] signal = {};
    private int windowSize;
    private int halfWindowSize;
    private double[] filteredSignal;

    public SlidingMedianFilter( int windowSize) {
        if (signal == null || signal.length == 0 || windowSize <= 0 || windowSize % 2 == 0) {
            throw new IllegalArgumentException("Invalid input");
        }
        this.windowSize = windowSize;
        this.halfWindowSize = windowSize / 2;
        this.filteredSignal = new double[signal.length];
        initializeFilteredSignal();
    }

    private void initializeFilteredSignal() {
        for (int i = 0; i < signal.length; i++) {
            double[] window = getWindow(i);
            filteredSignal[i] = calculateMedian(window);
        }
    }

    private double[] getWindow(int index) {
        double[] window = new double[windowSize];
        int windowIndex = 0;
        for (int j = index - halfWindowSize; j <= index + halfWindowSize; j++) {
            if (j >= 0 && j < signal.length) {
                window[windowIndex++] = signal[j];
            }
        }
        return Arrays.copyOf(window, windowIndex);
    }

    private double calculateMedian(double[] arr) {
        Arrays.sort(arr);
        int n = arr.length;
        if (n % 2 == 0) {
            return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
        } else {
            return arr[n / 2];
        }
    }

    public double[] getFilteredSignal() {
        return filteredSignal;
    }

    public void addValue(double newValue) {
        double[] newSignal = Arrays.copyOf(signal, signal.length + 1);
        newSignal[signal.length] = newValue;
        signal = newSignal;

        double[] newFilteredSignal = Arrays.copyOf(filteredSignal, filteredSignal.length + 1);
        double[] window = getWindow(signal.length - 1);
        newFilteredSignal[filteredSignal.length] = calculateMedian(window);
        filteredSignal = newFilteredSignal;
    }

    public static void main(String[] args) {
        double[] signal = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        int windowSize = 3;
        SlidingMedianFilter filter = new SlidingMedianFilter( windowSize);

        // Print the initial filtered signal
        System.out.println("Initial Filtered Signal:");
        for (double value : filter.getFilteredSignal()) {
            System.out.print(value + " ");
        }
        System.out.println();

        // Add new values and print the updated filtered signal
        for (double value : signal) {
            filter.addValue(value);
            System.out.println("Filtered Signal after adding " + value + ":");
            for (double filteredValue : filter.getFilteredSignal()) {
                System.out.print(filteredValue + " ");
            }
            System.out.println();
        }
    }
}
