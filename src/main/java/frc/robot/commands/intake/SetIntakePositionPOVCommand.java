package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakePositionPOVCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private Joystick gamepad;

    public SetIntakePositionPOVCommand(IntakeSubsystem intake, Joystick gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;

        addRequirements(intake);
    }

    @Override
    public void execute() {

        // this apparently takes a while so might need to optimize?
        // maybe getPOV() takes a really long time? do we want to bind this to buttons instead?
        // but the climber is using all the buttons?
        if (gamepad.getPOV() == 0) {
            intake.setExtended(true);
        } else if (gamepad.getPOV() == 180) {
            intake.setExtended(false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
