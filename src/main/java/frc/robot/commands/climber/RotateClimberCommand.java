package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RotateClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private double angle;
    private boolean leftSide = false;

    public RotateClimberCommand(ClimberSubsystem climber, double angle, boolean leftSide) {
        this.climber = climber;
        this.angle = angle;
        this.leftSide = leftSide;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (leftSide) {
            climber.setLeftTilt(angle);
        } else {
            climber.setRightTilt(angle);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
