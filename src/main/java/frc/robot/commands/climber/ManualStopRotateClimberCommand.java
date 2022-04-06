package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualStopRotateClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private boolean leftSide;

    public ManualStopRotateClimberCommand(ClimberSubsystem climber, boolean leftSide) {
        this.climber = climber;
        this.leftSide = leftSide;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.stopClimber(leftSide);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
