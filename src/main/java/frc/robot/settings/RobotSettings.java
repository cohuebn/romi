package frc.robot.settings;

import edu.wpi.first.wpilibj.Preferences;

/**
 * A class to hold application preferences that can be tweaked at runtime
 * via the SmartDashboard, Shuffleboard, etc.
 */
public class RobotSettings {
    public static final String debugEnabledSetting = "debug";
    public static final String driveStraightTestDurationInSecondsSetting = "driveStraightTestDurationInSeconds";
    private static double driveStraightTestDurationInSecondsDefault = 10;
    public static final String driveStraightTestVelocityPercentageSetting = "driveStraightTestVelocityPercentage";
    private static double driveStraightTestVelocityPercentageDefault = 0.5;

    public static boolean debugEnabled() {
        return Preferences.getBoolean(debugEnabledSetting, false);
    }

    public static void setDebugEnabled(Boolean value) {
        Preferences.setBoolean(debugEnabledSetting, value);
    }

    public static double driveStraightTestTimeInSeconds() {
        return Preferences.getDouble(driveStraightTestDurationInSecondsSetting,
                driveStraightTestDurationInSecondsDefault);
    }

    public static void setDriveStraightTestTimeInSeconds(double value) {
        Preferences.setDouble(driveStraightTestDurationInSecondsSetting, value);
    }

    public static double driveStraightTestVelocityPercentage() {
        return Preferences.getDouble(driveStraightTestVelocityPercentageSetting,
                driveStraightTestVelocityPercentageDefault);
    }

    public static void setDriveStraightTestVelocityPercentage(double value) {
        Preferences.setDouble(driveStraightTestVelocityPercentageSetting, value);
    }

    /* Initialize preferences to give starting values at robot startup */
    public static void initializeRobotSettings() {
        RobotSettings.setDebugEnabled(false);
        RobotSettings.setDriveStraightTestTimeInSeconds(driveStraightTestDurationInSecondsDefault);
        RobotSettings.setDriveStraightTestVelocityPercentage(driveStraightTestVelocityPercentageDefault);
    }
}
