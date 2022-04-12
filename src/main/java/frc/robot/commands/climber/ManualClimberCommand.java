package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualClimberCommand extends CommandBase {
    private final ClimberSubsystem climber;
    private double power = 0;

    public ManualClimberCommand(ClimberSubsystem climber, double power) {
        this.climber = climber;
        this.power = power;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.manualExtend(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
