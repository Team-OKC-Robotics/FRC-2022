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
        // might want to only reset encoders when limit switches are pressed to a known value because 
        // the encoders are relative so starting position can mess things up
        // climber.resetEncoders();
    }

    @Override
    public void execute() {
        if (distance == -1) { // if we're not going to a specific distance
            climber.extend(leftSide); // just set it
        } else { // otherwise
            if (leftSide) {
                climber.setLeftExtend(distance); // set it to a specific distance
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
