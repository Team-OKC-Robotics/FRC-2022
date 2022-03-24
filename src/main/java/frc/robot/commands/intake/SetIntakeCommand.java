package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private double power;

    public SetIntakeCommand(IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void execute() {
        intake.setIntake(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
