package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeK;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax deployMotor;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexerMotor;

    private RelativeEncoder deployEncoder;
    private SparkMaxPIDController extendPID;

    private DigitalInput deployedLimitSwitch;
    private DigitalInput retractedLimitSwitch;
    private int direction = 0;
    private double intakePos = 0;

    private DigitalInput ballDetector;
    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatcher;
    private final Color blue;
    private final Color red;
    private Color currentAlliance;
    private int cargoCount = 0;
    private int indexerDirection = 0;
    private boolean now = false;
    private boolean lastBallDetector = false;
    private boolean reverseReverse = false;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("intake");
    private NetworkTableEntry writeMode = tab.add("write mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.add("intake ticks", 0).getEntry();
    private NetworkTableEntry velocity = tab.add("intake velocity", 0).getEntry();
    private NetworkTableEntry deployedSwitch = tab.add("deployed switch", false).getEntry();
    private NetworkTableEntry retractedSwitch = tab.add("retracted switch", false).getEntry();
    private NetworkTableEntry extended = tab.add("extended", false).getEntry();
    private NetworkTableEntry colorR = tab.add("color - r", 0).getEntry();
    private NetworkTableEntry colorG = tab.add("color - g", 0).getEntry();
    private NetworkTableEntry colorB = tab.add("color - b", 0).getEntry();
    
    // PID
    private NetworkTableEntry intakeP = tab.add("Intake kP", IntakeK.deployP).getEntry();
    private NetworkTableEntry intakeI = tab.add("Intake kI", IntakeK.deployI).getEntry();
    private NetworkTableEntry intakeD = tab.add("Intake kD", IntakeK.deployD).getEntry();

    private NetworkTableEntry deployedPreset = tab.add("Deployed preset", IntakeK.EXTENDED).getEntry();
            
    /**
     * makes a new IntakeSubsystem
     * the intake consists of the intake itself, which goes up and down, and the powered wheels to suck balls in
     * there's also a 'pass-through' or 'indexer' motor to move the balls along to the shooter
     */
    public IntakeSubsystem() {
        deployMotor = new CANSparkMax(10, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(7, MotorType.kBrushless); // not the shooter tower but the middle indexer
        intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
    
        if (deployMotor != null) {
            deployMotor.setIdleMode(IdleMode.kCoast);
            deployMotor.setOpenLoopRampRate(1); // oh so _that's_ why it was going to slow dang
            // this is so the intake doesn't kill something on the way down (or up)
            
            deployEncoder = deployMotor.getEncoder();
            deployEncoder.setPosition(0);

            extendPID = deployMotor.getPIDController();
            extendPID.setP(IntakeK.deployP);
            extendPID.setI(IntakeK.deployI);
            extendPID.setD(IntakeK.deployD);
        }

        if (indexerMotor != null) {
            indexerMotor.setIdleMode(IdleMode.kCoast);
            indexerMotor.setInverted(true);
            indexerMotor.setSmartCurrentLimit(30); // so as to not kill the baby neo
            indxerMotor.setOpenLoopRampRate(0.1) // so as not to kill the baby neo
        }

        if (intakeMotor != null) {
            intakeMotor.setIdleMode(IdleMode.kCoast);
            intakeMotor.setInverted(true);
            intakeMotor.setOpenLoopRampRate(0.1); // so as not to destroy the belts
        }

        deployedLimitSwitch = new DigitalInput(2);
        retractedLimitSwitch = new DigitalInput(3);
        ballDetector = new DigitalInput(8);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();
        blue = new Color(0, 0, 0);
        red = new Color(0, 0, 0);
        colorMatch.addColorMatch(red);
        colorMatch.addColorMatch(blue);
    }

    public void resetDeployEncoder() {
        deployEncoder.setPosition(0);
    }

    /**
     * Sets the intake powered wheels to the given power
     * @param power the power to set the intake to
     */
    public void setIntake(double power) {
        if (intakeMotor != null) {
            if (reverseReverse) {
                intakeMotor.set(-0.8);
            } else {
                intakeMotor.set(power);
            }
        }
    }

    /**
     * sets the indexer motor to the given power
     * but doesn't move if a ball is at the top
     * @param power the power to set the indexer to
     */
    public void setIndexer(double power) {
        if (indexerMotor != null) {
            if (reverseReverse) {
                indexerMotor.set(-1);
            } else {
                indexerMotor.set(power);
                indexerDirection = sign(power);
            }
        }
    }

    public int sign(double number) {
        if (number < 0) {
            return -1;
        } else if (number > 0) {
            return 1;
        }
        return 0;
    }

    /**
     * sets the indexer motor to the given power 
     * @param power the power to set the indexer to
     */
    public void feed(double power) {
        if (indexerMotor != null) {
            indexerMotor.set(power);
        }
    }

    public void manualDeploy(double power) {
        extendPID.setReference(power, ControlType.kDutyCycle);
    }

    public void manualStop() {
        intakePos = deployEncoder.getPosition();
        extendPID.setReference(intakePos, ControlType.kPosition);
    }

    /**
     * set the intake to be extended/deployed
     * @param extended if the intake should be extended/deployed or not
     */
    public void setExtended(boolean extended) {
        if (deployMotor != null) {
            if (extended) { // if we're going to deploy it
                deployPID.setSetpoint(deployedPreset.getDouble(IntakeK.EXTENDED)); // set the PID to deploy
                direction = 1; // set the direction we're going in (for limit switch purposes)
            } else {
                deployPID.setSetpoint(IntakeK.RAISED); // same thing
                direction = -1;
            }
        }
    }

    /**
     * returns if the intake is extended/deployed
     * @return true if the intake is extended
     */
    public boolean isExtended() {
        // return deployEncoder.getPosition() == IntakeK.EXTENDED;
        return true;
    }

    public void decreaseCargoCount() {
        cargoCount -= 1;
    }

    @Override
    public void periodic() {
        if (currentAlliance == null) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                currentAlliance = red;
            } else {
                currentAlliance = blue;
            }
        }


        now = !ballDetector.get(); // make it normal logic
        if (now && !lastBallDetector && indexerDirection == 1 && !deployedLimitSwitch.get()) { // if we have a ball, and we didn't previously have one, and we're moving it into the robot, and the intake is deployed
            ColorMatchResult color = colorMatcher.matchClosestColor(colorSensor.getColor()); // get the color of the ball
            if (color == currentAlliance) { // if it's one we want
                cargoCount += 1; // raise the cargo count
                if (cargoCount > 2) { // if we are carrying more than two
                    reverseReverse = true; // spit it out
                    setIntake(-0.8);
                    setIndexer(-0.8);
                } else { // otherwise we're good
                    reverseReverse = false; // and can keep on going normally
                }
            } else { // otherwise it's not our color of cargo
                reverseReverse = true; // so spit it out
            }
        } else if (!now && lastBallDetector) { // if we don't have the cargo anymore
            reverseReverse = false; // then we can stop spitting out (might need to something so it makes it run for a certain period of time or something)
        }
        lastBallDetector = now; // update the variable for the last state

        if (cargoCount == 2) { // if we have 2 cargo
            deployPID.setSetpoint(IntakeK.RAISED); // raise the intake because we're not going to be intaking anymore
        }

        // I feel like there's potential for some speedup here by combining these if statements
        if (!deployedLimitSwitch.get()) { // if limit switch is pressed
            deployEncoder.setPosition(IntakeK.EXTENDED); // set the intake encoder to the correct position
        } else if (!retractedLimitSwitch.get()) { // if the other limit switch is pressed
            deployEncoder.setPosition(0); // then it's at 0
        }
           
        if (direction != 0) { // don't start moving unless the code has started and the intake has been told to move,
                              // so it can be moved when powered on but disabled
            if (!deployedLimitSwitch.get() && direction == -1) { // if the limit switch is pressed
                deployed.setBoolean(true);
                deployMotor.set(0); // stop the intake
            } else if (!retractedLimitSwitch.get() && direction == 1) { // if the retracted limit switch is pressed
                deployed.setBoolean(false);
                deployMotor.set(0); // stop the intake
            } else { // otherwise we're good to keep moving
                double power = deployPID.calculate(deployEncoder.getPosition()); // calculate the power
                if (Math.abs(power) > 0.4) { // limit the power to a max of 0.4
                    power = Math.copySign(0.4, power);
                }
                deployMotor.set(power); // move the intake
            }
        }
        
        if (!Constants.competition) {
            if (deployEncoder != null) {
                extended.setBoolean(isExtended());
                ticks.setDouble(deployEncoder.getPosition());
                velocity.setDouble(deployEncoder.getVelocity());
            }
    
            if (writeMode.getBoolean(false)) {
                if (extendPID != null) {
                    extendPID.setP(intakeP.getDouble(IntakeK.deployP));
                    extendPID.setI(intakeI.getDouble(IntakeK.deployI));
                    extendPID.setD(intakeD.getDouble(IntakeK.deployD));
                }
            }
            
            if (deployedLimitSwitch.get()) {
                deployedSwitch.setBoolean(false);
            } else {
                deployedSwitch.setBoolean(true);
            }
            
            if (retractedLimitSwitch.get()) {
                retractedSwitch.setBoolean(false);
            } else {
                retractedSwitch.setBoolean(true);
            }

            RawColor color = colorSensor.getRawColor();
            colorR.setDouble(color.red);
            colorG.setDouble(color.green);
            colorB.setDouble(color.blue);
        }
    }
}