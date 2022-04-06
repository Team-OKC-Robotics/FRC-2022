package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetSpeedDrive extends WaitCommand {
    private final DrivetrainSubsystem drivetrain;
    private double setSpeed = 1;

    /**
     * Drives the drivetrain on the heading it had when the command started, for the given distance
     * @param drivetrain the drivetrain subsystem for the command to control
     * @param distance the distance, in inches, to drive the robot
     */
    public SetSpeedDrive(DrivetrainSubsystem drivetrain, double setSpeed, double time) {
        super(time);
        this.drivetrain = drivetrain;
        this.setSpeed = setSpeed;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(setSpeed, 0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
