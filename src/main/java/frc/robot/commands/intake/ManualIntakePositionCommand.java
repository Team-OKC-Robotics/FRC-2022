package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakePositionCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private boolean extended;

    public ManualIntakePositionCommand(IntakeSubsystem intake, boolean extended) {
        this.intake = intake;
        this.extended = extended;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        // if (extended) {
        //     intake.incrementIntakePosition();
        // } else {
        //     intake.decrementIntakePosition();
        // }
        if (extended) {
            intake.manualDeploy(-0.5);
        } else {
            intake.manualDeploy(0.5);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
