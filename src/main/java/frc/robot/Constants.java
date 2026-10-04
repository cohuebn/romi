// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final LinearVelocityUnit drivetrainVelocityUnit = Units.InchesPerSecond;
    // Theoretical max velocity of Romi at full charge
    public static final LinearVelocity maxDrivetrainVelocity = drivetrainVelocityUnit.of(25);
    // Max voltage to feed to the Romi; the Romi supports a maximum voltage of 10.8V. However, not going
    // quite that high to keep things the nominal safe range of voltage for the system. This also is the
    // max voltage the rechargable AA batteries can achieve
    public static final Voltage maxDrivetrainVoltage = Volts.of(7.2);
    // Tuned feed-forward constants for the drivetrain motors
    public static final double leftMotorFeedForwardKS = 0.211;
    public static final double leftMotorFeedForwardKV = 0.2605;
    public static final double rightMotorFeedForwardKS = 0.295;
    public static final double rightMotorFeedForwardKV = 0.275;
}
