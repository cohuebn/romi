// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class ArcadeDrive extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Double> xAxisSpeedSupplier;
  private final Supplier<Double> zAxisRotateSupplier;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xAxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zAxisRotateSupplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xAxisSpeedSupplier,
      Supplier<Double> zAxisRotateSupplier) {
    this.drivetrain = drivetrain;
    this.xAxisSpeedSupplier = xAxisSpeedSupplier;
    this.zAxisRotateSupplier = zAxisRotateSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDriveVelocity(xAxisSpeedSupplier.get(), zAxisRotateSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
