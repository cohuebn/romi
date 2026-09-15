// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.RobotSettings;
import frc.robot.utilities.Constraints;

public class Drivetrain extends SubsystemBase {
  private static final double encoderCountsPerWheelRevolution = 1440.0;
  private static final Distance wheelDiameter = Units.Millimeter.of(70); // 2.75591 in.
  // Theoretical max velocity of Romi at full charge
  private static final LinearVelocity maxVelocity = InchesPerSecond.of(35);

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  // private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);

  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetPIDControllers() {
    leftMotorPID.reset();
    rightMotorPID.reset();
  }

  // The PID controllers are used to account for differences in left/right motor
  // powers
  // to allow automatic correction based on distances reported by the encoders
  // TODO - tune PID controller values
  private final PIDController leftMotorPID = new PIDController(0.02, 0.0, 0.0);
  private final PIDController rightMotorPID = new PIDController(0.02, 0.0, 0.0);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // SendableRegistry.addChild(differentialDrive, leftMotor);
    // SendableRegistry.addChild(differentialDrive, rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    double distancePerEncoderPulse = Math.PI * wheelDiameter.in(Units.Inches) / encoderCountsPerWheelRevolution;
    leftEncoder.setDistancePerPulse(distancePerEncoderPulse);
    rightEncoder.setDistancePerPulse(distancePerEncoderPulse);

    resetEncoders();
  }

  // public void arcadeDrive(double xAxisSpeed, double zAxisRotate) {
  //   differentialDrive.arcadeDrive(xAxisSpeed, zAxisRotate);
  // }

  /**
   * Using a target velocity from a controller and recorded velocities from the
   * encoders,
   * determine how much power to give each motor to achieve the desired velocity
   */
  public void driveAtDesiredVelocity(double desiredLeftVelocity, double desiredRightVelocity) {
    // Current measured wheel velocities.
    double leftMeasuredVelocity = leftEncoder.getRate();
    double rightMeasuredVelocity = rightEncoder.getRate();

    // PID calculates the motor output needed to reach the desired velocity.
    double leftOutput = leftMotorPID.calculate(leftMeasuredVelocity, desiredLeftVelocity);
    double rightOutput = rightMotorPID.calculate(rightMeasuredVelocity, desiredRightVelocity);

    // Keep the motor commands within the valid range.
    leftOutput = Constraints.clamp(leftOutput, -1.0, 1.0);
    leftMotor.set(leftOutput);
    rightOutput = Constraints.clamp(rightOutput, -1.0, 1.0);
    rightMotor.set(rightOutput);
  }

  private double normalizeArcadeInput(double input) {
    return MathUtil.applyDeadband(Math.pow(input, 2), 0.02);
  }

  /** Convert joystick/controller values into desired wheel velocities */
  public void arcadeDriveVelocity(double xAxisSpeed, double zAxisRotate) {
    double maxVelocityMeasure = maxVelocity.in(Units.InchesPerSecond);
    double xAxisSpeedWithDeaband = normalizeArcadeInput(xAxisSpeed);
    double zAxisRotateWithDeaband = normalizeArcadeInput(zAxisRotate);
    double leftVelocity = (xAxisSpeedWithDeaband + zAxisRotateWithDeaband) * maxVelocityMeasure;
    double rightVelocity = (xAxisSpeedWithDeaband - zAxisRotateWithDeaband) * maxVelocityMeasure;

    /*
     * Arcade drive can produce values greater than 1.0 when x and z
     * are both large. Scale them back down while preserving their ratio.
     */
    double max = Math.max(Math.abs(leftVelocity), Math.abs(rightVelocity));

    if (max > maxVelocityMeasure) {
      leftVelocity = leftVelocity / max * maxVelocityMeasure;
      rightVelocity = rightVelocity / max * maxVelocityMeasure;
    }

    driveAtDesiredVelocity(leftVelocity, rightVelocity);
  }

  public Distance getLeftDistance() {
    return Units.Inches.of(leftEncoder.getDistance());
  }

  public Distance getRightDistance() {
    return Units.Inches.of(rightEncoder.getDistance());
  }

  public Distance getAverageDistance() {
    return getLeftDistance().plus(getRightDistance()).div(2.0);
  }

  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Record useful encoder measurements for observing and tuning
   */
  private void recordEncoderMeasurements() {
    SmartDashboard.putNumber("leftEncoderDistanceInches", getLeftDistance().in(Units.Inches));
    SmartDashboard.putNumber("rightEncoderDistanceInches", getRightDistance().in(Units.Inches));
    double leftEncoderRate = leftEncoder.getRate();
    SmartDashboard.putNumber("leftEncoderRate", leftEncoderRate);
    double rightEncoderRate = leftEncoder.getRate();
    SmartDashboard.putNumber("rightEncoderRate", rightEncoderRate);
    // A difference of zero would indicate both wheels move at the same rate
    SmartDashboard.putNumber("encoderRateDifference", leftEncoderRate - rightEncoderRate);
  }

  @Override
  public void periodic() {
    // Only record encoder measurements in debug mode to avoid latency outside of
    // debugging
    if (RobotSettings.debugEnabled()) {
      recordEncoderMeasurements();
    }
  }
}
