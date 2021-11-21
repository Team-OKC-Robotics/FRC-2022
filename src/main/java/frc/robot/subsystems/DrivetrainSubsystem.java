package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    private SpeedControllerGroup leftSide;
    private WPI_TalonFX left1Motor;
    //private WPI_TalonFX left2Motor;

    private WPI_TalonFX right1Motor;
    //private WPI_TalonFX right2Motor;
    private SpeedControllerGroup rightSide;

    private DifferentialDrive drivetrain;
    //TODO add built-in Rio accelerometer to this subsystem
    //TODO add shuffleboard support

    public DrivetrainSubsystem() {
        left1Motor = new WPI_TalonFX(1);
        //left2Motor = new WPI_TalonFX(3);
        leftSide = new SpeedControllerGroup(left1Motor);

        right1Motor = new WPI_TalonFX(2);
        //right2Motor = new WPI_TalonFX(1);
        rightSide = new SpeedControllerGroup(right1Motor);

        drivetrain = new DifferentialDrive(leftSide, rightSide);
    }

    /**
     * Tank drives the robot. Auto-squares the inputs.
     * @param leftSpeed left speed
     * @param rightSpeed right speed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed / 1, rightSpeed / 1, false);
    }

    /**
     * Arcade drives the robot. Does not square the inputs.
     * @param speed the speed of the robot
     * @param turn how much to turn the robot
     */
    public void arcadeDrive(double speed, double turn) {
        drivetrain.arcadeDrive(speed, turn, false);
    }
    

}
