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
import frc.robot.Constants.DriveK;

public class DrivetrainSubsystem extends SubsystemBase {
    // actuators
    private WPI_TalonFX left1Motor;
    private WPI_TalonFX right1Motor;
    
    private SpeedControllerGroup leftSide;
    private SpeedControllerGroup rightSide;

    private DifferentialDrive drivetrain;

    // sensors
    private BuiltInAccelerometer gyro;
    
    // PID controllers
    private PIDController distancePID;
    private PIDController headingPID;
    private PIDController turnPID;
    
    // other variables
    public double speedModifier = 1; // the speed modifier for the drivetrain (the joystick input is multiplied by this value)

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("drivetrain");
    private NetworkTableEntry writeMode = tab.addPersistent("Write Mode", false).getEntry();
    private NetworkTableEntry leftTicks = tab.addPersistent("left ticks", 0).getEntry();
    private NetworkTableEntry rightTicks = tab.addPersistent("right ticks", 0).getEntry();
    private NetworkTableEntry totalTicks = tab.addPersistent("total ticks", 0).getEntry();
    private NetworkTableEntry distanceP = tab.addPersistent("Distance kP", 0).getEntry();
    private NetworkTableEntry distanceD = tab.addPersistent("Distance kD", 0).getEntry();

    public DrivetrainSubsystem() {
        // motor configuration
        left1Motor = new WPI_TalonFX(1);
        //left2Motor = new WPI_TalonFX(3);
        leftSide = new SpeedControllerGroup(left1Motor);

        right1Motor = new WPI_TalonFX(2);
        //right2Motor = new WPI_TalonFX(1);
        rightSide = new SpeedControllerGroup(right1Motor);

        left1Motor.setNeutralMode(NeutralMode.Brake);
        right1Motor.setNeutralMode(NeutralMode.Brake);

        drivetrain = new DifferentialDrive(leftSide, rightSide);

        // sensor configuration
        gyro = new BuiltInAccelerometer();

        // PID configuration
        distancePID = new PIDController(DriveK.distanceP, DriveK.distanceI, DriveK.distanceD);
        headingPID = new PIDController(0, 0, 0);
        turnPID = new PIDController(0, 0, 0);

        // Shuffleboard initilization
        leftTicks.setDouble(0);
        rightTicks.setDouble(0);
        totalTicks.setDouble(0);

        // reset the subsystem
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

    /**
     * Resets the encoders and the corresponding Shuffleboard entries
     */
    public void resetEncoders() {
        right1Motor.getSensorCollection().setIntegratedSensorPosition(0, 500);
        left1Motor.getSensorCollection().setIntegratedSensorPosition(0, 500);
        totalTicks.setDouble(0);
        leftTicks.setDouble(0);
        rightTicks.setDouble(0);
    }

    /**
     * Converts ticks from encoders into inches using the constants in Constants.DriveK
     * @param encoderTicks the number of ticks to convert
     * @return the number of inches that the ticks is equal to
     */
    public double getInches(double encoderTicks) {
        return encoderTicks / DriveK.ticksPerRev * DriveK.gearRatio * Math.PI * DriveK.wheelDiameter;
    }

    /**
     * Gets if the distancePID is at its setpoint
     * @return true if the distancePID is at its setpoing
     */
    public boolean atDistanceSetpoint() {
        return distancePID.atSetpoint();
    }

    /**
     * Resets the distance PID
     */
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
        // update Shuffelboard values
        leftTicks.setDouble(getLeftEncoderAverage());
        rightTicks.setDouble(getRightEncoderAverage());
        totalTicks.setDouble(getEncoderAverage());

        // Shuffleboard on-the-fly tuning
        if (writeMode.getBoolean(false)) {
            distancePID.setP(distanceP.getDouble(DriveK.distanceP));
            distancePID.setD(distanceD.getDouble(DriveK.distanceD));
        }
    }
}
