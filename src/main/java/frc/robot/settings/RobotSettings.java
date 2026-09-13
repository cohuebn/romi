package frc.robot.settings;

import edu.wpi.first.wpilibj.Preferences;

/** A class to hold application preferences that can be tweaked at runtime
 * via the SmartDashboard, Shuffleboard, etc. */
public class RobotSettings {
    public static final String debugEnabledSetting = "debug";

    public static boolean debugEnabled() {
        return Preferences.getBoolean(debugEnabledSetting, false);
    }

    public static void setDebugEnabled(Boolean value) {
        Preferences.setBoolean(debugEnabledSetting, value);
    }
}
