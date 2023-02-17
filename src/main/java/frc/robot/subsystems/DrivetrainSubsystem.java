package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveK;

public class DrivetrainSubsystem extends SubsystemBase {
    // actuators
    private Victor left1Motor;
    private Victor left2Motor;

    private Victor right1Motor;
    private Victor right2Motor;
    
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
        left1Motor = new Victor(1);
        left2Motor = new Victor(2);
        leftSide = new MotorControllerGroup(left1Motor, left2Motor);

        right1Motor = new Victor(3);
        right2Motor = new Victor(4);
        rightSide = new MotorControllerGroup(right1Motor, right2Motor);

        rightSide.setInverted(true); // motors face opposite directions so +1 for left side is opposite +1 for right side, this fixes that
        drivetrain = new DifferentialDrive(leftSide, rightSide);

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

        resetDistancePID();
        resetHeadingPID();
        resetTurnPID();
        resetGyro();
    }

    public void setSpeedModifier(double speedMod) {
        speedModifier = speedMod;
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
