package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private double angle = 0;
    private double maxOutput = 1;

    /**
     * Turns the drivetrain by the given angle (ie this is a relative turn)
     * @param drivetrain the drivetrain subsystem for the command to control
     * @param angle the angle, in degrees, to turn the robot by
     */
    public TurnCommand(DrivetrainSubsystem drivetrain, double angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;

        addRequirements(drivetrain);
    }

    public TurnCommand(DrivetrainSubsystem drivetrain, double angle, double maxOutput) {
        this.drivetrain = drivetrain;
        this.angle = angle;
        this.maxOutput = maxOutput;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetTurnPID();
        drivetrain.resetEncoders();
        drivetrain.resetGyro();
        drivetrain.setMaxOutput(maxOutput);
    }

    @Override
    public void execute() {
        drivetrain.turnToHeading(angle);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atTurnSetpoint();
    }
}
