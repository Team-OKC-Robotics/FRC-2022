package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class StopCommand extends CommandBase {
    private final TestSubsystem subsystem;

    public StopCommand(TestSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        subsystem.manualStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
