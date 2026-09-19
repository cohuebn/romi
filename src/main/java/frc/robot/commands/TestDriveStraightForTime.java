// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.settings.RobotSettings;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public class TestDriveStraightForTime extends Command {
  private final Drivetrain drivetrain;
  private long startTime;

  /**
   * Creates a new TestDriveStraightForTime. This command will drive your robot straight
   * using RobotSettings to determine velocity and time. It is primarily used for testing
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public TestDriveStraightForTime(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    drivetrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity velocity = Constants.maxDrivetrainVelocity.times(RobotSettings.driveStraightTestVelocityPercentage());
    double velocityInDrivetrainUnits = velocity.in(Constants.drivetrainVelocityUnit); 
    drivetrain.driveAtDesiredVelocity(velocityInDrivetrainUnits, velocityInDrivetrainUnits);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= RobotSettings.driveStraightTestTimeInSeconds() * 1000;
  }
}
