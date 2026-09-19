package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

enum DriveCalibrationPhase {
    RampUp,
    RampDown,
    ReturnToZero
}

/**
 * Command used for drivetrain calibration; this command steps up through volts
 * to monitor
 * the impact on the motor encoders. Then when max voltage hits, it steps back
 * down until
 * zero volts is hit
 */
public class DriveCalibration extends Command {
    private static final String dashboardGroup = "Drivetrain Calibration";
    private Drivetrain drivetrain;
    private Voltage voltage;
    private Voltage voltageStep;
    private Instant calibrationStepStartTime;
    private Duration calibrationStepDuration;
    private DriveCalibrationPhase calibrationPhase;

    public DriveCalibration(Voltage voltageStep, Duration calibrationStepDuration, Drivetrain drivetrain) {
        this.voltageStep = voltageStep;
        this.calibrationStepDuration = calibrationStepDuration;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    /**
     * Initialize the calibration command. This resets encoders and sets initial
     * values for drivetrain control to start calibration
     */
    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        drivetrain.stop();
        voltage = Volts.of(0);
        calibrationStepStartTime = Instant.now();
        calibrationPhase = DriveCalibrationPhase.RampUp;
    }

    private boolean hasStaticFrictionBeenOvercome(LinearVelocity currentVelocity) {
        // Not setting threshold to absolute zero, because encoders have a little noise
        // that indicate velocity is above zero even at zero volts
        return currentVelocity.gt(Constants.drivetrainVelocityUnit.of(0.0000001));
    }

    private String dashboardMetricKey(String key) {
        return dashboardGroup + "/" + key;
    }

    /** Record the results of a voltage change to the dashboard */
    private void recordResults() {
        SmartDashboard.putString(dashboardMetricKey("phase"), calibrationPhase.name());
        SmartDashboard.putNumber(dashboardMetricKey("targetVoltage"), getTargetVoltage().abs(Volts));
        SmartDashboard.putNumber(dashboardMetricKey("voltage"), voltage.abs(Volts));
        SmartDashboard.putNumber(dashboardMetricKey("leftMotorRate"), drivetrain.getLeftEncoderRateAsDouble());
        SmartDashboard.putBoolean(dashboardMetricKey("leftMotorStaticFrictionOvercome"),
                hasStaticFrictionBeenOvercome(drivetrain.getLeftEncoderRate()));
        SmartDashboard.putNumber(dashboardMetricKey("rightMotorRate"), drivetrain.getRightEncoderRateAsDouble());
        SmartDashboard.putBoolean(dashboardMetricKey("rightMotorStaticFrictionOvercome"),
                hasStaticFrictionBeenOvercome(drivetrain.getRightEncoderRate()));
    }

    private Voltage getTargetVoltage() {
        if (calibrationPhase == DriveCalibrationPhase.RampUp)
            return Constants.maxDrivetrainVoltage;
        if (calibrationPhase == DriveCalibrationPhase.RampDown)
            return Constants.maxDrivetrainVoltage.unaryMinus();
        return Volts.zero();
    }

    private void changeCalibrationPhaseWhenVoltageMet() {
        if (calibrationPhase == DriveCalibrationPhase.RampUp && voltage.gte(getTargetVoltage())) {
            calibrationPhase = DriveCalibrationPhase.RampDown;
        }
        if (calibrationPhase == DriveCalibrationPhase.RampDown && voltage.lte(getTargetVoltage())) {
            calibrationPhase = DriveCalibrationPhase.ReturnToZero;
        }
    }

    /**
     * Potentially increase voltage and update the previous calibration step values
     * for continued calibration
     */
    private void prepareForNextCalibrationExecution() {
        boolean timeForVoltageChange = Duration.between(calibrationStepStartTime, Instant.now())
                .compareTo(calibrationStepDuration) > 0;
        if (timeForVoltageChange) {
            changeCalibrationPhaseWhenVoltageMet();
            voltage = calibrationPhase == DriveCalibrationPhase.RampDown ? voltage.minus(voltageStep)
                    : voltage.plus(voltageStep);
            calibrationStepStartTime = Instant.now();
        }
    }

    /**
     * Run the calibration process. This involves slowly incresing motor values and
     * observing behavior and recording results for observation
     */
    @Override
    public void execute() {
        drivetrain.setMotorVoltage(voltage, voltage);
        recordResults();
        prepareForNextCalibrationExecution();
    }

    @Override
    public boolean isFinished() {
        return calibrationPhase == DriveCalibrationPhase.ReturnToZero && voltage.equals(Volts.zero());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        recordResults();
        drivetrain.resetEncoders();
    }
}
