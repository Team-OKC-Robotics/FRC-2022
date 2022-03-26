package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualStopIntakePositionCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public ManualStopIntakePositionCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        // if (extended) {
        //     intake.incrementIntakePosition();
        // } else {
        //     intake.decrementIntakePosition();
        // }
        intake.manualStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
