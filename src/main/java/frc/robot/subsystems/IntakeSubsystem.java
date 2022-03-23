package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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
    private PIDController deployPID;

    private DigitalInput deployedLimitSwitch;
    private DigitalInput retractedLimitSwitch;
    private int direction = 0;
    private DigitalInput ballDetector;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("intake");
    private NetworkTableEntry writeMode = tab.add("write mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.add("intake ticks", 0).getEntry();
    private NetworkTableEntry velocity = tab.add("intake velocity", 0).getEntry();
    private NetworkTableEntry deployedSwitch = tab.add("deployed switch", false).getEntry();
    private NetworkTableEntry retractedSwitch = tab.add("retracted switch", false).getEntry();
    private NetworkTableEntry extended = tab.add("extended", false).getEntry();
    private NetworkTableEntry hasBall = tab.add("has ball?", false).getEntry();
    
    // PID
    private NetworkTableEntry intakeP = tab.add("Intake kP", IntakeK.deployP).getEntry();
    private NetworkTableEntry intakeI = tab.add("Intake kI", IntakeK.deployI).getEntry();
    private NetworkTableEntry intakeD = tab.add("Intake kD", IntakeK.deployD).getEntry();

    private NetworkTableEntry deployedPreset = tab.add("Deployed preset", IntakeK.EXTENDED).getEntry();
    
    // I don't think there needs to be any shuffleboard stuff here
    // we could do some weird stuff with like hasBall() but that's not important right now
        
    /**
     * makes a new IntakeSubsystem
     * the intake consists of the intake itself, which goes up and down, and the powered wheels to suck balls in
     * there's also a 'pass-through' or 'indexer' motor to move the balls along to the shooter
     */
    public IntakeSubsystem() {
        deployMotor = new CANSparkMax(10, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(9, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
    
        if (deployMotor != null) {
            extendPID = deployMotor.getPIDController(); //TODO configure this because it's gonna not work right because going down is gonna kill stuff
            deployEncoder = deployMotor.getEncoder();
            deployMotor.setSoftLimit(SoftLimitDirection.kForward, IntakeK.maxDeploy);
            deployMotor.setIdleMode(IdleMode.kCoast);
            deployEncoder.setPosition(0);

            extendPID.setP(IntakeK.deployP);
            extendPID.setI(IntakeK.deployI);
            extendPID.setD(IntakeK.deployD);
        }

        if (indexerMotor != null) {
            indexerMotor.setIdleMode(IdleMode.kBrake);
        }

        if (intakeMotor != null) {
            intakeMotor.setIdleMode(IdleMode.kCoast);
        }

        deployPID = new PIDController(IntakeK.deployP, IntakeK.deployI, IntakeK.deployD);
        deployedLimitSwitch = new DigitalInput(2);
        retractedLimitSwitch = new DigitalInput(3);
        ballDetector = new DigitalInput(9);
    }

    /**
     * Sets the intake powered wheels to the given power
     * @param power the power to set the intake to
     */
    public void setIntake(double power) {
        // if (intakeMotor != null) {
            intakeMotor.set(power);
        // }
    }

    /**
     * sets the indexer motor to the given power
     * but doesn't move if a ball is at the top
     * @param power the power to set the indexer to
     */
    public void setIndexer(double power) {
        if (indexerMotor != null) {
            if (!ballDetector.get()) { // ball detector is inverse logic, so if we have ball
                if (power <= 0) { // don't let the ball move forwards
                    indexerMotor.set(power);
                } else {
                    indexerMotor.set(0);
                }
            } else {
                indexerMotor.set(power); // otherwise run as much as you want
            }
        }
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

    /**
     * set the intake to be extended/deployed
     * @param extended if the intake should be extended/deployed or not
     */
    public void setExtended(boolean extended) {
        if (deployMotor != null) {
            if (extended) {
                deployPID.setSetpoint(deployedPreset.getDouble(IntakeK.EXTENDED));
                direction = 1;
            } else {
                deployPID.setSetpoint(IntakeK.RAISED);
                direction = -1;
            }
        }

        // if (extendPID != null) {
        //     if (extended) {
        //         extendPID.setReference(deployedPreset.getDouble(IntakeK.EXTENDED), ControlType.kPosition);
        //     } else {
        //         extendPID.setReference(IntakeK.RAISED, ControlType.kPosition);
        //     }
        // }
    }

    /**
     * returns if the intake is extended/deployed
     * @return true if the intake is extended
     */
    public boolean isExtended() {
        // return deployEncoder.getPosition() == IntakeK.EXTENDED;
        return true;
    }

    @Override
    public void periodic() {
        if (!Constants.competition) {
            hasBall.setBoolean(ballDetector.get());
            if (deployedLimitSwitch.get()) {
                deployedSwitch.setBoolean(true);
            } else {
                deployedSwitch.setBoolean(false);
            }
            
            if (retractedLimitSwitch.get()) {
                retractedSwitch.setBoolean(true);
            } else {
                retractedSwitch.setBoolean(false);
            }
        }

        // I feel like there's potential for some speedup here by combining these if statements
        if (!deployedLimitSwitch.get()) {
            deployEncoder.setPosition(IntakeK.EXTENDED);
        } else if (!retractedLimitSwitch.get()) {
            deployEncoder.setPosition(0);
        }
        // retracted limit switch is reversed logic
        // deployed is reversed aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
           
        if (direction != 0) {
            if (!deployedLimitSwitch.get() && direction == -1) {
                // deployed.setBoolean(true);
                deployMotor.set(0);
            } else if (!retractedLimitSwitch.get() && direction == 1) {
                // deployed.setBoolean(false);
                deployMotor.set(0);
            }
            double power = deployPID.calculate(deployEncoder.getPosition());
            if (Math.abs(power) > 0.4) {
                power = Math.copySign(0.4, power);
            }
            deployMotor.set(power);
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
        }
    }
}