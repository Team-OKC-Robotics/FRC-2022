package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualRotateClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private Joystick gamepad;
    private boolean leftSide;

    public ManualRotateClimberCommand(ClimberSubsystem climber, Joystick gamepad, boolean leftSide) {
        this.climber = climber;
        this.gamepad = gamepad;
        this.leftSide = leftSide;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.manualTilt(gamepad.getRawAxis(1) * 0.2, leftSide);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
