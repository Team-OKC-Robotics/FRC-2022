package frc.robot.util;

public class FrcUtil {
    public static double clamp(double minOutput, double maxOutput, double input) {
        if (input < minOutput) {
            return minOutput;
        } else if (input > maxOutput) {
            return maxOutput;
        } else {
            return input;
        }
    }

}
