package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetIdleCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public SetIdleCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake, shooter);
    }

    @Override
    public void execute() {
        // if (intake.hasBall()) {
        //     shooter.setIdle();
        //     intake.setIdle();
        // } else {
        //     intake.setIndexer(0.3);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
