package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

public class BilinearInterpolator {
    private final double[] dists;  // X-axis (e.g., 48, 60, 72...)
    private final double[] vels;   // Y-axis (e.g., 1800, 1900, 2000...)
    private final double[][] table; // Z-values (Angles)

    public BilinearInterpolator(double[] dists, double[] vels, double[][] table) {
        this.dists = dists;
        this.vels = vels;
        this.table = table;
    }

    public double getAngle(double currentDist, double currentVel) {
        // 1. Find the "cell" we are in (the four surrounding points)
        int dIdx = findLowerIndex(dists, currentDist);
        int vIdx = findLowerIndex(vels, currentVel);

        // 2. Define the four corner values from your table
        // table[row][column] -> table[velocity][distance]
        double z11 = table[vIdx][dIdx];         // Bottom-Left
        double z12 = table[vIdx][dIdx + 1];     // Bottom-Right
        double z21 = table[vIdx + 1][dIdx];     // Top-Left
        double z22 = table[vIdx + 1][dIdx + 1]; // Top-Right

        // 3. Calculate "t" and "u" (percentage distance between points, 0.0 to 1.0)
        double t = (currentDist - dists[dIdx]) / (dists[dIdx + 1] - dists[dIdx]);
        double u = (currentVel - vels[vIdx]) / (vels[vIdx + 1] - vels[vIdx]);

        // 4. The Bilinear Formula
        // First, interpolate horizontally along the bottom and top
        double bottomLerp = z11 + t * (z12 - z11);
        double topLerp    = z21 + t * (z22 - z21);

        // Then, interpolate vertically between those two results
        return bottomLerp + u * (topLerp - bottomLerp);
    }

    private int findLowerIndex(double[] arr, double val) {
        // Simple search to find the lower bound index
        for (int i = 0; i < arr.length - 1; i++) {
            if (val < arr[i + 1]) return i;
        }
        return arr.length - 2; // Clip to the second-to-last index if out of bounds
    }
}