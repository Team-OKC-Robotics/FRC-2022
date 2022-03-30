package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class SetLEDsCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final boolean on;

    public SetLEDsCommand(VisionSubsystem vision, boolean on) {
        this.vision = vision;
        this.on = on;

        addRequirements(vision);
    }

    @Override
    public void execute() {
        vision.setLeds(on);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}