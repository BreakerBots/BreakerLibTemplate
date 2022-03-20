package frc.robot.BreakerLib.Util;

import java.util.function.IntToDoubleFunction;
import java.util.function.ToDoubleFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

//easily accessible conversion equations
public class BreakerMath {

    private static double prevTime = 0;

    // Drive logistic curve constants

    /** L constant for logistic curve */
    private static double L = 0.9;
    /** k constant for logistic curve */
    private static double k = 7.5;
    /** x0 constant for logistic curve */
    private static double x0 = 0.6;
    /** Vertical translation for logistic curve */
    private static double b = 0.15;

    /**
     * Constrains an angle value in degrees within +- 360 degrees.
     * 
     * @param deg Angle value in degrees.
     * 
     * @return Angle value within -360 to +360 degrees.
     */
    public static final double angleModulus(double deg) {
        return angleModulus(deg, 360);
    }

    /**
     * Constrains an angle value in degrees within +- desired constraint, in degrees
     * 
     * @param deg        Angle value in degrees.
     * @param constraint Degree value to constrain angle within.
     * 
     * @return Angle value within -constraint to +constraint degrees.
     */
    public static final double angleModulus(double deg, double constraint) {
        return deg % constraint;
    }

    /**
     * Gets the amount of time in seconds between cycles.
     * 
     * @return Time difference between cycles, in seconds.
     */
    public static double getCycleDiffTime() {
        double curTime = RobotController.getFPGATime(); // In microseconds
        double diffTime = curTime - prevTime;
        prevTime = curTime;
        diffTime = BreakerUnits.microsecondsToSeconds(diffTime); // Value converted to seconds
        return diffTime;
    }

    public static double getCircumferenceFromRadus(double radius) {
        return (2 * radius) * Math.PI;
    }

    public static double getCircumferenceFromDiameter(double diameter) {
        return diameter * Math.PI;
    }

    public static double getTicksPerRotation(double encoderTicks, double gearRatioTo1) {
        return encoderTicks * gearRatioTo1;
    }

    public static double getTicksPerInch(double encoderTicks, double gearRatioTo1, double wheelDiameter) {
        return getTicksPerRotation(encoderTicks, gearRatioTo1) / getCircumferenceFromDiameter(wheelDiameter);
    }

    /**
     * @param ticks Talon FX encoder ticks.
     * @return distance, in inches.
     */
    public static double ticksToInches(double ticks, double ticksPerInch) {
        return ticks / ticksPerInch;
    }

    /**
     * Returns y when x is fed into pre-determined logistic curve.
     * 
     * @param x Value between -1 and 1.
     * @return Value between -1 and 1.
     */
    public static double driveCurve(double x) {
        double absX = MathUtil.applyDeadband(Math.abs(x), 0.05);
        double y = (Math.signum(x) * L) / (1 + Math.pow(Math.E, -k * (absX - x0))) + b;
        return y;
    }

    public static double rollingAvg(double avg, double newVal) {
        return (avg + newVal) / 2.0;
    }

    public static double getAvg(double lastAvg, double newVal, int cycleCount) {
        return (((lastAvg * (cycleCount - 1)) + newVal) / cycleCount);
    }

    public static double fixedToFloat(int FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    public static double fixedToFloat(Long FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    public static double fixedToFloat(Short FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }
}
