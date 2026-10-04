// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  // The feed-forward controllers are used to preemptively determine voltage
  private final SimpleMotorFeedforward leftMotorFeedForward = new SimpleMotorFeedforward(
      Constants.leftMotorFeedForwardKS, Constants.leftMotorFeedForwardKV);
  private final SimpleMotorFeedforward rightMotorFeedForward = new SimpleMotorFeedforward(
      Constants.rightMotorFeedForwardKS,
      Constants.rightMotorFeedForwardKV);

  // The PID controllers are used to account for differences in left/right motor
  // powers to allow automatic correction based on distances reported by the
  // encoders
  private final PIDController leftMotorPID = new PIDController(0.2, 0.0, 0.0);
  private final PIDController rightMotorPID = new PIDController(0.2, 0.0, 0.0);

  // Desired velocity measurements are useful to see how well the controllers are
  // matching
  // the expected outcome
  private LinearVelocity latestLeftMotorDesiredVelocity;
  private LinearVelocity latestRightMotorDesiredVelocity;

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetPIDControllers() {
    leftMotorPID.reset();
    rightMotorPID.reset();
  }

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.add(leftEncoder, "Left Motor Encoder");
    SendableRegistry.add(rightEncoder, "Right Motor Encoder");
    SendableRegistry.add(leftMotorPID, "Left Motor PID");
    SendableRegistry.add(rightMotorPID, "Right Motor PID");

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

  /**
   * Using a target velocity from a controller and recorded velocities from the
   * encoders,
   * determine how much power to give each motor to achieve the desired velocity
   */
  public void driveAtDesiredVelocity(double desiredLeftVelocity, double desiredRightVelocity) {
    // Store desired velocities for later telemetry recording
    latestLeftMotorDesiredVelocity = Constants.drivetrainVelocityUnit.of(desiredLeftVelocity);
    latestRightMotorDesiredVelocity = Constants.drivetrainVelocityUnit.of(desiredRightVelocity);

    // Current measured wheel velocities.
    double leftMeasuredVelocity = leftEncoder.getRate();
    double rightMeasuredVelocity = rightEncoder.getRate();

    // PID calculates offsets between desired and actual velocities
    double leftPIDCorrection = leftMotorPID.calculate(leftMeasuredVelocity, desiredLeftVelocity);
    // double leftPIDCorrection = 0;
    double rightPIDCorrection = rightMotorPID.calculate(rightMeasuredVelocity, desiredRightVelocity);
    // double rightPIDCorrection = 0;

    // Use the feed-forward to estimate volts needed to hit desired velocity; use
    // the PID correction to account for non-modeled voltage changes needed
    double leftVolts = leftMotorFeedForward.calculate(desiredLeftVelocity) + leftPIDCorrection;
    double rightVolts = rightMotorFeedForward.calculate(desiredRightVelocity) + rightPIDCorrection;

    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  private double normalizeArcadeInput(double input) {
    // Squaring input gives finer control at small values
    double squaredInput = Math.pow(input, 2);
    // Deadband prevents joystick drift from moving motors
    double withDeadbandApplied = MathUtil.applyDeadband(squaredInput, 0.02);
    // Preserve the direction of the input
    return input < 0 ? -withDeadbandApplied : withDeadbandApplied;
  }

  /** Convert joystick/controller values into desired wheel velocities */
  public void arcadeDriveVelocity(double xAxisSpeed, double zAxisRotate) {
    double xAxisSpeedWithDeaband = normalizeArcadeInput(xAxisSpeed);
    double zAxisRotateWithDeaband = normalizeArcadeInput(zAxisRotate);
    double maxVelocityMeasure = Constants.maxDrivetrainVelocity.in(Constants.drivetrainVelocityUnit);
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

  /**
   * Normally, you wouldn't use this method to directly set voltage on motors.
   * However,
   * for controller calibration, it is often useful to set left/right motor
   * voltages
   * directly and observe results
   */
  public void setMotorVoltage(Voltage leftVoltage, Voltage rightVoltage) {
    leftMotor.setVoltage(leftVoltage.in(Volts));
    rightMotor.setVoltage(rightVoltage.in(Volts));
  }

  public void stop() {
    driveAtDesiredVelocity(0, 0);
  }

  // Encoder properties used for observability in commands for controllers,
  // tuning, etc.
  public LinearVelocity getLeftEncoderRate() {
    return Constants.drivetrainVelocityUnit.of(leftEncoder.getRate());
  }

  public double getLeftEncoderRateAsDouble() {
    return leftEncoder.getRate();
  }

  public Distance getLeftDistance() {
    return Units.Inches.of(leftEncoder.getDistance());
  }

  public LinearVelocity getRightEncoderRate() {
    return Constants.drivetrainVelocityUnit.of(rightEncoder.getRate());
  }

  public double getRightEncoderRateAsDouble() {
    return rightEncoder.getRate();
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

  private void recordOptionalVelocity(String key, LinearVelocity velocity) {
    if (velocity != null) {
      SmartDashboard.putNumber(key, velocity.in(Constants.drivetrainVelocityUnit));
    }
  }

  /**
   * Record useful encoder measurements for observing and tuning
   */
  private void recordMeasurements() {
    SmartDashboard.putData(leftEncoder);
    SmartDashboard.putData(leftMotorPID);
    SmartDashboard.putData(rightEncoder);
    SmartDashboard.putData(rightMotorPID);
    recordOptionalVelocity("latestLeftMotorDesiredVelocity", latestLeftMotorDesiredVelocity);
    recordOptionalVelocity("latestRightMotorDesiredVelocity", latestRightMotorDesiredVelocity);
    SmartDashboard.putNumber("leftMotorVoltage", leftMotor.getVoltage());
    SmartDashboard.putNumber("rightMotorVoltage", rightMotor.getVoltage());
  }

  @Override
  public void periodic() {
    // Only record encoder measurements in debug mode to avoid added latency outside
    // of debugging
    if (RobotSettings.debugEnabled()) {
      recordMeasurements();
    }
  }
}
