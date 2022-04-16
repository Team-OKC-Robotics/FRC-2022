package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.DriverStation;
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
        vision.setLeds(true);
        drivetrain.setOpenLoopRampRate(0.1);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0, vision.getOutput());
    }

    @Override
    public void end(boolean interrupted) {
        // if (interrupted) {
        //     DriverStation.reportWarning("we are interrupted", false);
        // }
        // DriverStation.reportWarning("yes the end() is getting called", false);
        drivetrain.arcadeDrive(0, 0);
        vision.setLeds(false);
        drivetrain.setOpenLoopRampRate();
    }
    
    @Override
    public boolean isFinished() {
        return vision.atVisionSetpoint();
    }
}