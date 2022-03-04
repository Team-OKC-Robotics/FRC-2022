package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private double distance = -1;
    private boolean leftSide = false;

    public ExtendClimberCommand(ClimberSubsystem climber, double distance, boolean leftSide) {
        this.climber = climber;
        this.distance = distance;
        this.leftSide = leftSide;

        addRequirements(climber);
    }

    public ExtendClimberCommand(ClimberSubsystem climber, boolean leftSide) {
        this.climber =  climber;
        this.leftSide = leftSide;
    }

    @Override
    public void initialize() {
        //TODO maybe reset encoders here?
    }

    @Override
    public void execute() {
        if (distance == -1) {
            climber.extend(leftSide);
        } else {
            if (leftSide) {
                climber.setLeftExtend(distance);
            } else {
                climber.setRightExtend(distance);
            }
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
