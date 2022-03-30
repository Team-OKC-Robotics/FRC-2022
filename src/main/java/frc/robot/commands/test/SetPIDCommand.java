package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class SetPIDCommand extends CommandBase {
    private final TestSubsystem subsystem;
    private double value = 0;

    public SetPIDCommand(TestSubsystem subsystem, double value) {
        this.subsystem = subsystem;
        this.value = value;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        subsystem.setPID(value);
    }

    @Override
    public void end(boolean executed) {
        subsystem.finish();
    }

    @Override
    public boolean isFinished() {
        return subsystem.atSetpoint();
    }
}
