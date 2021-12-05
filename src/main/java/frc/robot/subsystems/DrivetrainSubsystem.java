package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private SpeedControllerGroup leftSide;
    private WPI_TalonFX left1Motor;

    private WPI_TalonFX right1Motor;
    
    private SpeedControllerGroup rightSide;

    private DifferentialDrive drivetrain;

    private PIDController distancePID;
    private PIDController headingPID;
    private PIDController turnPID;

    private BuiltInAccelerometer gyro;
    //TODO add shuffleboard support

    public DrivetrainSubsystem() {
        left1Motor = new WPI_TalonFX(1);
        //left2Motor = new WPI_TalonFX(3);
        leftSide = new SpeedControllerGroup(left1Motor);

        right1Motor = new WPI_TalonFX(2);
        //right2Motor = new WPI_TalonFX(1);
        rightSide = new SpeedControllerGroup(right1Motor);

        drivetrain = new DifferentialDrive(leftSide, rightSide);

        gyro = new BuiltInAccelerometer();

        distancePID = new PIDController(0.5, 0, 0);
        headingPID = new PIDController(0, 0, 0);
        turnPID = new PIDController(0, 0, 0);
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
    
    /**
     * Drives the drivetrain straight for the given distance
     * @param distance the distance, in inches, to drive forwards
     */
    public void driveDistance(double distance) {
        distancePID.setSetpoint(distance);

        arcadeDrive(distancePID.calculate(getInches(getEncoderAverage())), headingPID.calculate(getHeading()));
    }

    /**
     * Gets the average of both sides of the drivetrain
     * @return the average distance of the drivetrain, in inches
     */
    public double getEncoderAverage() {
        return (getLeftEncoderAverage() + getRightEncoderAverage()) / 2;
    }

    /**
     * Gets the average of the left side of the drivetrain
     * @return the average distance of the left side of the drivetrain, in inches
     */
    public double getLeftEncoderAverage() {
        return left1Motor.getSensorCollection().getIntegratedSensorAbsolutePosition();
    }

    /**
     * Gets the average of the right side of the drivetrain
     * @return the average distance of the right side of the drivetrain, in inches
     */
    public double getRightEncoderAverage() {
        return -right1Motor.getSensorCollection().getIntegratedSensorAbsolutePosition();
    }

    public void resetEncoders() {
        right1Motor.getSensorCollection().setIntegratedSensorPosition(0, 200);
        left1Motor.getSensorCollection().setIntegratedSensorPosition(0, 200);
    }

    public double getInches(double encoderTicks) {
        return encoderTicks / Constants.ticksPerRev * Constants.gearRatio * Math.PI * Constants.wheelDiameter;
    }

    public boolean atDistanceSetpoint() {
        return distancePID.atSetpoint();
    }

    /**
     * Gets the heading of the built-in roboRIO gyro, in degrees, 0-360
     * @return the heading of the gyro
     */
    public double getHeading() {
        //return gyro.get
        return 0;
    }
}
