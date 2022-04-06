package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ResetIntakeEncoderCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public ResetIntakeEncoderCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.resetDeployEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
