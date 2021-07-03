package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    private Victor left1Motor;
    private Victor left2Motor;
    private SpeedControllerGroup leftSide;

    private Victor right1Motor;
    private Victor right2Motor;
    private SpeedControllerGroup rightSide;

    private DifferentialDrive drivetrain;

    public DrivetrainSubsystem() {
        left1Motor = new Victor(0);
        left2Motor = new Victor(1);
        leftSide = new SpeedControllerGroup(left1Motor, left2Motor);

        right1Motor = new Victor(2);
        right2Motor = new Victor(3);
        rightSide = new SpeedControllerGroup(right1Motor, right2Motor);

        drivetrain = new DifferentialDrive(leftSide, rightSide);
    }

    /**
     * Tank drives the robot
     * @param leftSpeed left speed
     * @param rightSpeed right speed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Arcade drives the robot
     * @param speed the speed of the robot
     * @param turn how much to turn the robot
     */
    public void arcadeDrive(double speed, double turn) {
        drivetrain.arcadeDrive(speed, turn, false);
    }
    
}
