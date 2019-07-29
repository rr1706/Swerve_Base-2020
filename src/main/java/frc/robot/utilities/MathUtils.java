package frc.robot.utilities;

public class MathUtils {

    private MathUtils() {
        throw new IllegalStateException("Utility Class");
    }

    public static Pair<Double, Double> convertOrientation(double currentHeading, double fwd, double str) {
        double x;
        double y;
        currentHeading = Math.toRadians(currentHeading);

        x = (str * Math.cos(currentHeading)) - (fwd * Math.sin(currentHeading)); // STR
        y = (fwd * Math.cos(currentHeading)) + (str * Math.sin(currentHeading)); // FWD

        return new Pair<>(x, y);
    }

    // Returns the equivalent value between two different ranges
    public static double convertRange(double oldMin, double oldMax, double newMin, double newMax, double oldValue) {
        double oldRange = (oldMax - oldMin);
        double newRange = (newMax - newMin);

        return (((oldValue - oldMin) * newRange) / oldRange) + newMin;
    }

    public static double pythagorean(double a, double b) {
        return Math.sqrt((Math.pow(a, 2) + Math.pow(b, 2)));
    }

    public static double resolveAngle(double deg) {
        while (deg > 360.0) {
            deg -= 360.0;
        }

        while (deg < 0.0) {
            deg += 360.0;
        }

        return deg;
    }
}
