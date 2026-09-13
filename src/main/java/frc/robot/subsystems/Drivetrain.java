// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.RobotSettings;

public class Drivetrain extends SubsystemBase {
  private static final double encoderCountsPerWheelRevolution = 1440.0;
  private static final Distance wheelDiameter = Units.Millimeter.of(70); // 2.75591 in.

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);

  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(differentialDrive, leftMotor);
    SendableRegistry.addChild(differentialDrive, rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * wheelDiameter.in(Units.Inches)) / encoderCountsPerWheelRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * wheelDiameter.in(Units.Inches)) / encoderCountsPerWheelRevolution);

    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    differentialDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }


  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Record useful encoder measurements for observing and tuning
   */
  private void recordEncoderMeasurements() {
    SmartDashboard.putNumber("leftEncoderDistanceInches", getLeftDistanceInch());
    SmartDashboard.putNumber("rightEncoderDistanceInches", getRightDistanceInch());
    SmartDashboard.putNumber("leftEncoderRate", leftEncoder.getRate());
    SmartDashboard.putNumber("rightEncoderRate", rightEncoder.getRate());
  }

  @Override
  public void periodic() {
    // Only record encoder measurements in debug mode to avoid latency outside of debugging
    if (RobotSettings.debugEnabled()) {
      recordEncoderMeasurements();
    }
  }
}
