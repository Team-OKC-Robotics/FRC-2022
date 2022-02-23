package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private double distance = 0;
    private boolean leftSide = false;

    public ExtendClimberCommand(ClimberSubsystem climber, double distance, boolean leftSide) {
        this.climber = climber;
        this.distance = distance;
        this.leftSide = leftSide;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        //TODO maybe reset encoders here?
    }

    @Override
    public void execute() {
    // public void initialize() {
        if (leftSide) {
            climber.setLeftExtend(distance);
        } else {
            climber.setRightExtend(distance);
        }
    }

    @Override
    public boolean isFinished() {
        if (leftSide) {
            return climber.atLeftExtendSetpoint();
        } else {
            return climber.atRightExtendSetpoint();
        }
    }
}
