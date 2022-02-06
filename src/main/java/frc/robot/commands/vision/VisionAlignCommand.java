package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;

    public VisionAlignCommand(VisionSubsystem vision, DrivetrainSubsystem drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        vision.resetVisionPID();
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0, vision.getOutput());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return vision.atVisionSetpoint();
    }
}
