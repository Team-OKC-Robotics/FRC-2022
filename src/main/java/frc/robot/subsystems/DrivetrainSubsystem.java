package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private WPI_TalonFX left1Motor;
    private WPI_TalonFX right1Motor;
    
    private SpeedControllerGroup leftSide;
    private SpeedControllerGroup rightSide;

    private DifferentialDrive drivetrain;

    private PIDController distancePID;
    private PIDController headingPID;
    private PIDController turnPID;

    public double speedModifier = 1;

    private BuiltInAccelerometer gyro;

    private ShuffleboardTab tab = Shuffleboard.getTab("drivetrain");
    private NetworkTableEntry leftTicks = tab.addPersistent("left ticks", 0).getEntry();
    private NetworkTableEntry rightTicks = tab.addPersistent("right ticks", 0).getEntry();
    private NetworkTableEntry totalTicks = tab.addPersistent("total ticks", 0).getEntry();
    //private NetworkTableEntry distanceP = tab.addPersistent("Distance kP", 0).withWidget(widgetType).getEntry();
    private NetworkTableEntry distanceD = tab.addPersistent("Distance kD", 0).getEntry();

    public DrivetrainSubsystem() {
        left1Motor = new WPI_TalonFX(1);
        //left2Motor = new WPI_TalonFX(3);
        leftSide = new SpeedControllerGroup(left1Motor);

        right1Motor = new WPI_TalonFX(2);
        //right2Motor = new WPI_TalonFX(1);
        rightSide = new SpeedControllerGroup(right1Motor);

        left1Motor.setNeutralMode(NeutralMode.Brake);
        right1Motor.setNeutralMode(NeutralMode.Brake);

        drivetrain = new DifferentialDrive(leftSide, rightSide);

        gyro = new BuiltInAccelerometer();

        distancePID = new PIDController(0.1, 0, 0.0001);
        headingPID = new PIDController(0, 0, 0);
        turnPID = new PIDController(0, 0, 0);

        leftTicks.setDouble(0);
        rightTicks.setDouble(0);
        totalTicks.setDouble(0);
        resetEncoders();
        resetDistancePID();
    }

    /**
     * Tank drives the robot. Auto-squares the inputs.
     * @param leftSpeed left speed
     * @param rightSpeed right speed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed * speedModifier, rightSpeed * speedModifier, false);
    }

    /**
     * Arcade drives the robot. Does not square the inputs.
     * @param speed the speed of the robot
     * @param turn how much to turn the robot
     */
    public void arcadeDrive(double speed, double turn) {
        drivetrain.arcadeDrive(speed * speedModifier, turn * speedModifier, false);
    }

    /**
     * Arcade drives the robot. Does not square the inputs.
     * @param speed the speed of the robot
     * @param turn how much to turn the robot
     * @param maxOutput the max output of the drivtrain
     */
    public void arcadeDrive(double speed, double turn, double maxOutput) {
        drivetrain.setMaxOutput(maxOutput);
        drivetrain.arcadeDrive(speed * speedModifier, turn * speedModifier, false);
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
        return left1Motor.getSensorCollection().getIntegratedSensorPosition();
    }

    /**
     * Gets the average of the right side of the drivetrain
     * @return the average distance of the right side of the drivetrain, in inches
     */
    public double getRightEncoderAverage() {
        return -right1Motor.getSensorCollection().getIntegratedSensorPosition();
    }

    public void resetEncoders() {
        right1Motor.getSensorCollection().setIntegratedSensorPosition(0, 500);
        left1Motor.getSensorCollection().setIntegratedSensorPosition(0, 500);
        totalTicks.setDouble(0);
        leftTicks.setDouble(0);
        rightTicks.setDouble(0);
    }

    public double getInches(double encoderTicks) {
        return encoderTicks / Constants.ticksPerRev * Constants.gearRatio * Math.PI * Constants.wheelDiameter;
    }

    public boolean atDistanceSetpoint() {
        return distancePID.atSetpoint();
    }

    public void resetDistancePID() {
        distancePID.reset();
    }

    /**
     * Gets the heading of the built-in roboRIO gyro, in degrees, 0-360
     * @return the heading of the gyro
     */
    public double getHeading() {
        //return gyro.get
        return 0;
    }

    /**
     * Our periodic function, gets called every robot loop iteration
     * Updates shuffleboard values.
     */
    @Override
    public void periodic() {
        leftTicks.setDouble(getLeftEncoderAverage());
        rightTicks.setDouble(getRightEncoderAverage());
        totalTicks.setDouble(getEncoderAverage());

        // if (distanceP.getDouble(0) != distancePID.getP()) {
        //     distancePID.setP(distanceP.getDouble(0));
        // }

        // if (distanceD.getDouble(0) != distancePID.getD()) {
        //     distancePID.setD(distanceD.getDouble(0));
        // }
    }
}
