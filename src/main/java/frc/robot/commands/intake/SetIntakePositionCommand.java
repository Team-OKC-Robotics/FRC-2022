package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakePositionCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private boolean extended;

    public SetIntakePositionCommand(IntakeSubsystem intake, boolean extended) {
        this.intake = intake;
        this.extended = extended;
    }

    @Override
    public void execute() {
        intake.setExtended(extended);
    }

    @Override
    public boolean isFinished() {
        return intake.isExtended() == extended;
    }
}
