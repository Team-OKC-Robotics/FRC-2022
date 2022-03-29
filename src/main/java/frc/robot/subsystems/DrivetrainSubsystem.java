package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveK;

public class DrivetrainSubsystem extends SubsystemBase {
    // actuators
    private CANSparkMax left1Motor;
    private CANSparkMax left2Motor;
    private CANSparkMax left3Motor;

    private CANSparkMax right1Motor;
    private CANSparkMax right2Motor;
    private CANSparkMax right3Motor;

    private RelativeEncoder left1Encoder;
    private RelativeEncoder left2Encoder;
    private RelativeEncoder left3Encoder;

    private RelativeEncoder right1Encoder;
    private RelativeEncoder right2Encoder;
    private RelativeEncoder right3Encoder;
    
    private MotorControllerGroup leftSide;
    private MotorControllerGroup rightSide;

    private DifferentialDrive drivetrain;

    // sensors
    private AHRS gyro; // the NavX gyro
    
    // PID controllers
    private PIDController distancePID;
    private PIDController headingPID;
    private PIDController turnPID;
    
    // other variables
    private double speedModifier = 0.75; // the speed modifier for the drivetrain (the joystick input is multiplied by this value)
    //private double headingAngle = 0; // the heading of the robot. used to drive straight in auto.
    private double openLoopRampRate = 0.5;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("drivetrain");
    private NetworkTableEntry writeMode = tab.add("Write Mode", false).getEntry();
    
    // distance
    private NetworkTableEntry leftTicks = tab.add("left ticks", 0).getEntry();
    private NetworkTableEntry rightTicks = tab.add("right ticks", 0).getEntry();
    private NetworkTableEntry totalTicks = tab.add("total ticks", 0).getEntry();
    private NetworkTableEntry distanceError = tab.add("distance error", 0).getEntry();
    
    private NetworkTableEntry distanceP = tab.add("Distance kP", DriveK.distanceP).getEntry();
    private NetworkTableEntry distanceI = tab.add("Distance kI", DriveK.distanceI).getEntry();
    private NetworkTableEntry distanceD = tab.add("Distance kD", DriveK.distanceD).getEntry();

    // angle
    private NetworkTableEntry heading = tab.add("heading", 0).getEntry();
    private NetworkTableEntry headingP = tab.add("Heading kP", DriveK.headingP).getEntry();
    private NetworkTableEntry headingI = tab.add("Heading kI", DriveK.headingI).getEntry();
    private NetworkTableEntry headingD = tab.add("Heading kD", DriveK.headingD).getEntry();

    private NetworkTableEntry turnP = tab.add("Turn kP", DriveK.turnP).getEntry();
    private NetworkTableEntry turnI = tab.add("Turn kI", DriveK.turnI).getEntry();
    private NetworkTableEntry turnD = tab.add("Turn kD", DriveK.turnD).getEntry();

    private NetworkTableEntry resetGyro = tab.add("reset gyro", false).getEntry();

    public DrivetrainSubsystem() {
        // motor configuration
        left1Motor = new CANSparkMax(1, MotorType.kBrushless);
        left2Motor = new CANSparkMax(2, MotorType.kBrushless);
        left3Motor = new CANSparkMax(3, MotorType.kBrushless);
        leftSide = new MotorControllerGroup(left1Motor, left2Motor, left3Motor);

        right1Motor = new CANSparkMax(4, MotorType.kBrushless);
        right2Motor = new CANSparkMax(5, MotorType.kBrushless);
        right3Motor = new CANSparkMax(6, MotorType.kBrushless);
        rightSide = new MotorControllerGroup(right1Motor, right2Motor, right3Motor);

        // coast mode so we don't kill the gearbox and motors (also makes driving easier)
        left1Motor.setIdleMode(IdleMode.kCoast);
        left2Motor.setIdleMode(IdleMode.kCoast);
        left3Motor.setIdleMode(IdleMode.kCoast);
        right1Motor.setIdleMode(IdleMode.kCoast);
        right2Motor.setIdleMode(IdleMode.kCoast);
        right3Motor.setIdleMode(IdleMode.kCoast);

        // for autonomous start with a fast ramp rate (not too fast otherwise we kind of break the gearboxes)
        // we had problems with stripping gears and whatnot because we put too much force on the gears so this limits
        // that. VEX guy said on Chief Delphi not to put too much acceleration on them otherwise they strip
        left1Motor.setOpenLoopRampRate(0.01);
        left2Motor.setOpenLoopRampRate(0.01);
        left3Motor.setOpenLoopRampRate(0.01);
        right1Motor.setOpenLoopRampRate(0.01);
        right2Motor.setOpenLoopRampRate(0.01);
        right3Motor.setOpenLoopRampRate(0.01);

        rightSide.setInverted(true); // motors face opposite directions so +1 for left side is opposite +1 for right side, this fixes that
        drivetrain = new DifferentialDrive(leftSide, rightSide);

        // get the built-in encoders of the NEOs       
        left1Encoder = left1Motor.getEncoder();
        left2Encoder = left2Motor.getEncoder();
        left3Encoder = left3Motor.getEncoder();
    
        right1Encoder = right1Motor.getEncoder();
        right2Encoder = right2Motor.getEncoder();
        right3Encoder = right3Motor.getEncoder();

        // sensor configuration
        gyro = new AHRS(SPI.Port.kMXP); // plugged into the big port thing on the RoboRIO
        gyro.calibrate();

        // PID configuration
        distancePID = new PIDController(DriveK.distanceP, DriveK.distanceI, DriveK.distanceD);
        headingPID = new PIDController(DriveK.headingP, DriveK.headingI, DriveK.headingD);
        turnPID = new PIDController(DriveK.turnP, DriveK.turnI, DriveK.turnD);

        // set the tolerance of the various PIDs. this ensures we don't wait for them to be super precise before
        // moving on (which in some cases without a kI term can be literally forever) but still ensures
        // we've settled down close enough
        distancePID.setTolerance(2);
        headingPID.setTolerance(7, 1);
        turnPID.setTolerance(7, 2);

        // Shuffleboard initilization of sensor values
        leftTicks.setDouble(0);
        rightTicks.setDouble(0);
        totalTicks.setDouble(0);
        heading.setDouble(0);

        // reset the subsystem
        resetEncoders();
        resetDistancePID();
        resetHeadingPID();
        resetTurnPID();
        resetGyro();
    }

    public void setOpenLoopRampRate() {
        left1Motor.setOpenLoopRampRate(openLoopRampRate);
        left2Motor.setOpenLoopRampRate(openLoopRampRate);
        left3Motor.setOpenLoopRampRate(openLoopRampRate);
        right1Motor.setOpenLoopRampRate(openLoopRampRate);
        right2Motor.setOpenLoopRampRate(openLoopRampRate);
        right3Motor.setOpenLoopRampRate(openLoopRampRate);
    }

    public void setOpenLoopRampRate(double rate) {
        left1Motor.setOpenLoopRampRate(rate);
        left2Motor.setOpenLoopRampRate(rate);
        left3Motor.setOpenLoopRampRate(rate);
        right1Motor.setOpenLoopRampRate(rate);
        right2Motor.setOpenLoopRampRate(rate);
        right3Motor.setOpenLoopRampRate(rate);
    }

    public void curvatureDrive(double speed, double turn, boolean turnInPlace) {
        drivetrain.curvatureDrive(speed, turn, turnInPlace);
    }

    /**
     * Tank drives the robot. Does not auto-square the inputs.
     * @param leftSpeed left speed
     * @param rightSpeed right speed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed * speedModifier, rightSpeed * speedModifier, true);
    }

    /**
     * Arcade drives the robot. Does not square the inputs.
     * @param speed the speed of the robot
     * @param turn how much to turn the robot
     */
    public void arcadeDrive(double speed, double turn) {
        drivetrain.arcadeDrive(speed * speedModifier, turn * speedModifier, true);
    }
    
    /**
     * Arcade drives the robot, taking into account speedModifier
     * @param speed the speed of the robot
     * @param turn how much to turn the robot
     * @param squareInputs whether or not to square the inputs for fine-grain control
     */
    public void arcadeDrive(double speed, double turn, boolean squareInputs) {
        drivetrain.arcadeDrive(speed * speedModifier, turn * speedModifier, squareInputs);
    }

    /**
     * Arcade drives the robot. Does not square the inputs.
     * @param speed the speed of the robot
     * @param turn how much to turn the robot
     */
    public void arcadeDriveAuto(double speed, double turn, boolean squareInputs) {
        drivetrain.arcadeDrive(-speed, turn, squareInputs);
    }

    /**
     * Drives the drivetrain straight for the given distance
     * @param distance the distance, in inches, to drive forwards
     */
    public void driveDistance(double distance) {
        distancePID.setSetpoint(distance);

        arcadeDrive(distancePID.calculate(getInches(getEncoderAverage())), headingPID.calculate(getHeading()));
    }

    public void driveOnHeading(double setSpeed, double distance) {
        distancePID.setSetpoint(distance);
        arcadeDriveAuto(clamp(-setSpeed, setSpeed, distancePID.calculate(getEncoderAverage())), headingPID.calculate(getHeading()), false);
    }

    public double clamp(double minOutput, double maxOutput, double input) {
        if (input < minOutput) {
            return minOutput;
        } else if (input > maxOutput) {
            return maxOutput;
        } else {
            return input;
        }
    }

    /**
     * Turns the robot _by_ the given heading
     * This is 'relative' turning, where passing 60 makes the robot turn 60 degrees instead of _to_ 60 degrees
     * @param heading the angle, in degrees, to turn the robot by (not sure which way is positive yet)
     */
    public void turnToHeading(double heading) {
        turnPID.setSetpoint(heading);

        arcadeDrive(0, turnPID.calculate(getHeading())); //TODO maybe change this so it tries to stay in the same spot with distancePID ???
    }

    /**
     * Stores the heading that the robot is currently on so it can PID to that in driveDistance()
     */
    public void setHeading() {
        // uh wait why is this here
        //headingAngle = getHeading();
        headingPID.setSetpoint(getHeading());
    }

    /**
     * Gets the average of both sides of the drivetrain
     * @return the average distance of the drivetrain, in inches
     */
    public double getEncoderAverage() {
        return (getLeftEncoderAverage() + -getRightEncoderAverage()) / 2;
    }

    /**
     * Gets the average of the left side of the drivetrain
     * @return the average distance of the left side of the drivetrain, in inches
     */
    public double getLeftEncoderAverage() {
        return left1Encoder.getPosition();
    }

    /**
     * Gets the average of the right side of the drivetrain
     * @return the average distance of the right side of the drivetrain, in inches
     */
    public double getRightEncoderAverage() {
        return right1Encoder.getPosition();
    }

    /**
     * Resets the encoders and the corresponding Shuffleboard entries
     */
    public void resetEncoders() {
        left1Encoder.setPosition(0);
        left2Encoder.setPosition(0);
        left3Encoder.setPosition(0);
        right1Encoder.setPosition(0);
        right2Encoder.setPosition(0);
        right3Encoder.setPosition(0);

        totalTicks.setDouble(0);
        leftTicks.setDouble(0);
        rightTicks.setDouble(0);
    }

    /**
     * Converts rotations from encoders into inches using the constants in Constants.DriveK
     * @param rotations the number of rotations to convert
     * @return the number of inches that the ticks is equal to
     */
    public double getInches(double rotations) {
        return rotations * DriveK.gearRatio * Math.PI * DriveK.wheelDiameter;
    }

    /**
     * Gets if the distancePID is at its setpoint
     * @return true if the distancePID is at its setpoing
     */
    public boolean atDistanceSetpoint() {
        return distancePID.atSetpoint();
    }

    /**
     * Gets if the headingPID is at its setpoint
     * @return true if the headingPID is at its setpoing
     */
    public boolean atHeadingSetpoint() {
        return headingPID.atSetpoint();
    }

    /**
     * Gets if the turnPID is at its setpoint
     * @return true if the turnPID is at its setpoing
     */
    public boolean atTurnSetpoint() {
        return turnPID.atSetpoint();
    }

    /**
     * Resets the distance PID
     */
    public void resetDistancePID() {
        distancePID.reset();
    }

    /**
     * Resets the heading PID
     */
    public void resetHeadingPID() {
        headingPID.reset();
    }

    /**
     * Resets the turn PID
     */
    public void resetTurnPID() {
        turnPID.reset();
    }

    /**
     * Gets the heading of our NavX, in degrees, 0-360 except it actually wraps infinitely so if it goes past 360 it goes to 361
     * @return the heading of the gyro
     */
    public double getHeading() {
        return gyro.getAngle();
        //return gyro.getQuaternionZ();
        //return gyro.getYaw();
        //return gyro.getAngle();
        //return gyro.getRotation2d().getDegrees(); // I think this is the right method but I'm not sure
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void setMaxOutput(double maxOutput) {
        drivetrain.setMaxOutput(maxOutput);
    }

    /**
     * Our periodic function, gets called every robot loop iteration
     * Updates shuffleboard values.
     */
    @Override
    public void periodic() {
        if (!Constants.competition) {
            // update Shuffelboard sensor values
            leftTicks.setDouble(getLeftEncoderAverage());
            rightTicks.setDouble(getRightEncoderAverage());
            totalTicks.setDouble(getEncoderAverage());
            heading.setDouble(getHeading());
            distanceError.setDouble(distancePID.getPositionError());
    
            // Shuffleboard on-the-fly tuning
            if (writeMode.getBoolean(false)) {
                distancePID.setP(distanceP.getDouble(DriveK.distanceP));
                distancePID.setI(distanceI.getDouble(DriveK.distanceI));
                distancePID.setD(distanceD.getDouble(DriveK.distanceD));
    
                headingPID.setP(headingP.getDouble(DriveK.headingP));
                headingPID.setI(headingI.getDouble(DriveK.headingI));
                headingPID.setD(headingD.getDouble(DriveK.headingD));
    
                turnPID.setP(turnP.getDouble(DriveK.turnP));
                turnPID.setI(turnI.getDouble(DriveK.turnI));
                turnPID.setD(turnD.getDouble(DriveK.turnD));
            }
        }
        
        // this needs to be there regardless
        if (resetGyro.getBoolean(false)) {
            resetGyro();
            resetGyro.setBoolean(false);
        }
    }
}
