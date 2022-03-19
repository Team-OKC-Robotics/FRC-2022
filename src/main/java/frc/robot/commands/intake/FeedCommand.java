package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class FeedCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private double power;

    public FeedCommand(IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.feed(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
