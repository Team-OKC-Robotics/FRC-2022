package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private double power = 0;
    private boolean leftSide;

    public ManualClimberCommand(ClimberSubsystem climber, double power, boolean leftSide) {
        this.climber = climber;
        this.power = power;
        this.leftSide = leftSide;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.manualExtend(power, leftSide);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
