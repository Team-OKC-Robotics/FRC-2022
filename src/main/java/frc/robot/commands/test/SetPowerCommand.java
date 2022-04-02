package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class SetPowerCommand extends CommandBase {
    private final TestSubsystem subsystem;
    private double value = 0;

    public SetPowerCommand(TestSubsystem subsystem, double value) {
        this.subsystem = subsystem;
        this.value = value;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        subsystem.setPower(value);
    }

    @Override
    public void end(boolean executed) {
        // subsystem.finish();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
