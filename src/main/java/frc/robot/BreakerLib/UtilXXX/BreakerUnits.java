package frc.robot.BreakerLib.UtilXXX;

import edu.wpi.first.math.util.Units;

public class BreakerUnits {
    public static final double MICROSECONDS_PER_SECOND = 1000000;
    public static final double METERS_PER_SECOND_SQUARED_IN_G = 9.80665;
    public static final double INCHES_PER_SECOND_SQUARED_IN_G = 386.088583;

    /**
     * Converts given microseconds to seconds.
     * 
     * @param microseconds The microseconds to convert to seconds.
     * 
     * @return Seconds converted from microseconds.
     */
    public static double microsecondsToSeconds(double microseconds) {
        return microseconds / MICROSECONDS_PER_SECOND;

    }

    /**
     * Converts given seconds to microseconds.
     * 
     * @param seconds The seconds to convert to microseconds.
     * 
     * @return Microseconds converted from seconds.
     */
    public static double secondsToMicroseconds(double seconds) {
        return seconds * MICROSECONDS_PER_SECOND;

    }

    /**
     * Converts given Gs to m/s^2.
     * 
     * @param gForces The Gs to convert to m/s^2.
     * 
     * @return m/s^2 converted from Gs.
     */
    public static double gForceToMetersPerSecondSquared(double gForces) {
        return gForces * METERS_PER_SECOND_SQUARED_IN_G;
    }

    /**
     * Converts given m/s^2 to Gs.
     * 
     * @param metersPerSecondSquared The m/s^2 to convert to Gs.
     * 
     * @return Gs converted from m/s^2.
     */
    public static double metersPerSecondSquaredToGs(double metersPerSecondSquared) {
        return metersPerSecondSquared / METERS_PER_SECOND_SQUARED_IN_G;
    }

    /**
     * Converts given Gs to in/s^2.
     * 
     * @param gForces The Gs to convert to in/s^2.
     * 
     * @return in/s^2 converted from Gs.
     */
    public static double gForceToInchesPerSecondSquared(double gForces) {
        return gForces * INCHES_PER_SECOND_SQUARED_IN_G;
    }

    /**
     * Converts given in/s^2 to Gs.
     * 
     * @param inchesPerSecondSquared The in/s^2 to convert to Gs.
     * 
     * @return Gs converted from in/s^2.
     */
    public static double inchesPerSecondSquaredToGs(double inchesPerSecondSquared) {
        return inchesPerSecondSquared / INCHES_PER_SECOND_SQUARED_IN_G;
    }
}