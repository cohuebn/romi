package frc.robot.utilities;

public class Constraints {
    /** Constrain the given value between the provided min and max values.
     * If the value is lest than the min, return the min
     * If the value is between the min and max, return the value
     * If the value is greater than the max, return the max
     * @param value The value to evaluate
     * @param min The minimum value allowed
     * @param max The maximum value allowed
     * @return The clamped value
    */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}
