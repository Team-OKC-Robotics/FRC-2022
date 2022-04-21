package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveSetSpeedCommand extends WaitCommand {
    private final DrivetrainSubsystem drivetrain;
    private double distance = 0;
    private double setSpeed = 1;

    /**
     * Drives the drivetrain on the heading it had when the command started, for the given distance
     * @param drivetrain the drivetrain subsystem for the command to control
     * @param distance the distance, in inches, to drive the robot
     */
    public DriveSetSpeedCommand(DrivetrainSubsystem drivetrain, double distance, double setSpeed, double timeout) {
        super(timeout);
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.setSpeed = setSpeed;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        drivetrain.resetDistancePID();
        drivetrain.resetHeadingPID();
        drivetrain.setHeading();
        drivetrain.resetGyro();
    }

    @Override
    public void execute() {
        drivetrain.driveOnHeading(setSpeed, distance);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atDistanceSetpoint() || super.isFinished();
    }
}
