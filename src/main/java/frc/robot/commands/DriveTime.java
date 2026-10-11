// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.settings.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveTime extends Command {
  private final double timeInSeconds;
  private final LinearVelocity velocity;
  private final Drivetrain drivetrain;
  private long startTime;

  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param velocity The velocity which the robot will drive. Negative is in reverse.
   * @param timeInSeconds How much time to drive in seconds
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public DriveTime(LinearVelocity velocity, double timeInSeconds, Drivetrain drivetrain) {
    this.velocity = velocity;
    this.timeInSeconds = timeInSeconds;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    drivetrain.arcadeDriveVelocity(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDriveVelocity(velocity.in(Constants.drivetrainVelocityUnit), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDriveVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= timeInSeconds * 1000;
  }
}
