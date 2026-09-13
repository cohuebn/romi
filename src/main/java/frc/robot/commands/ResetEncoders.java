package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * A command to reset values on encoders. This is typically useful while
 * debugging/troublshooting
 * to get a clean measurements from the encoders
 */
public class ResetEncoders extends Command {
    private Drivetrain drivetrain;

    public ResetEncoders(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public boolean runsWhenDisabled() {
        // The encoders can be reset in any mode, disabled included
        return true;
    }

    @Override
    public void execute() {
        drivetrain.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
