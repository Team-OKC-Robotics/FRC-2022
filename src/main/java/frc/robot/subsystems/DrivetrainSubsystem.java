package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.numbers.N2;
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

    private AHRS gyro;

    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

    //Simulation
    private PIDController leftPID;
    private PIDController rightPID;
    private Field2d fieldSim = new Field2d();
    private SimDevice gyroSim = SimDevice.create("ahrs", 0);
    private SimDeviceSim gyroSimSim = new SimDeviceSim("ahrs", 0);
    private Encoder leftDummyEncoder = new Encoder(0, 1);
    private Encoder rightDummyEncoder = new Encoder(2, 3);
    private EncoderSim leftEncoderSim = new EncoderSim(leftDummyEncoder);
    private EncoderSim rightEncoderSim = new EncoderSim(rightDummyEncoder);
    private LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(drivetrainSystem, DCMotor.getFalcon500(2), 8, Constants.trackWidth, 6, null);
    //private SimDouble simHeading = gyroSim.createDouble("heading", Direction.kBidir, 0);
    private int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    private SimDouble headingSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

    public DrivetrainSubsystem() {
        left1Motor = new WPI_TalonFX(1);
        //left2Motor = new WPI_TalonFX(3);
        leftSide = new SpeedControllerGroup(left1Motor);

        right1Motor = new WPI_TalonFX(2);
        //right2Motor = new WPI_TalonFX(1);
        rightSide = new SpeedControllerGroup(right1Motor);

        leftDummyEncoder.setDistancePerPulse(Math.PI * 2 * 6 / 2048);
        rightDummyEncoder.setDistancePerPulse(Math.PI * 2 * 6 / 2048);

        leftDummyEncoder.reset();
        rightDummyEncoder.reset();

        drivetrain = new DifferentialDrive(leftSide, rightSide);

        gyro = new AHRS(SPI.Port.kMXP); // TODO add correct port

        distancePID = new PIDController(0.5, 0, 0);
        headingPID = new PIDController(0, 0, 0);
        turnPID = new PIDController(0, 0, 0);
        
        leftPID = new PIDController(1, 0, 0);
        rightPID = new PIDController(1, 0, 0);

        SmartDashboard.putData("Field", fieldSim);
        headingSim.set(0);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        leftSide.setVoltage(leftPID.calculate(leftDummyEncoder.getRate(), speeds.leftMetersPerSecond));
        rightSide.setVoltage(rightPID.calculate(rightDummyEncoder.getRate(), speeds.rightMetersPerSecond));
    }

    public void drive(double xSpeed, double rotation) {
        setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rotation)));
    }

    public void updateOdemetry() {
        if (RobotBase.isSimulation()) {
            odometry.update(Rotation2d.fromDegrees(headingSim.get()), leftDummyEncoder.getDistance(), rightDummyEncoder.getDistance());
        } else {
            odometry.update(gyro.getRotation2d(), leftDummyEncoder.getDistance(), rightDummyEncoder.getDistance());
        }
    }

    public void resetOdemtry(Pose2d pose) {
        leftDummyEncoder.reset();
        rightDummyEncoder.reset();
        drivetrainSim.setPose(pose);
        if (RobotBase.isSimulation()) {
            odometry.resetPosition(pose, Rotation2d.fromDegrees(headingSim.get()));
        } else {
            odometry.resetPosition(pose, gyro.getRotation2d());
        }
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

    @Override
    public void simulationPeriodic() {
        drivetrainSim.setInputs(left1Motor.get() * RobotController.getInputVoltage(), -right1Motor.get() * RobotController.getInputVoltage());
        drivetrainSim.update(0.02);

        leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
        headingSim.set(-drivetrainSim.getHeading().getDegrees());
    }

    @Override
    public void periodic() {
        updateOdemetry();
        fieldSim.setRobotPose(odometry.getPoseMeters());
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
