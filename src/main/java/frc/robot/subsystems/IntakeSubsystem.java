package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
    private PIDController deployPID;

    private DigitalInput deployedLimitSwitch;
    private DigitalInput retractedLimitSwitch;
    private int direction = 0;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("intake");
    private NetworkTableEntry writeMode = tab.add("write mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.add("intake ticks", 0).getEntry();
    private NetworkTableEntry velocity = tab.add("intake velocity", 0).getEntry();
    private NetworkTableEntry deployedSwitch = tab.add("deployed switch", false).getEntry();
    private NetworkTableEntry retractedSwitch = tab.add("retracted switch", false).getEntry();
    private NetworkTableEntry extended = tab.add("extended", false).getEntry();
    
    // PID
    private NetworkTableEntry intakeP = tab.add("Intake kP", IntakeK.deployP).getEntry();
    private NetworkTableEntry intakeI = tab.add("Intake kI", IntakeK.deployI).getEntry();
    private NetworkTableEntry intakeD = tab.add("Intake kD", IntakeK.deployD).getEntry();

    private NetworkTableEntry deployedPreset = tab.add("Deployed preset", IntakeK.EXTENDED).getEntry();

    private DataLog log;
    private DoubleLogEntry posLog;
    private DoubleLogEntry outputLog;
    private DoubleLogEntry setpointLog;
            
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
            // deployMotor.setOpenLoopRampRate(0); // oh so _that's_ why it was going to slow dang
            // this is so the intake doesn't kill something on the way down (or up)
            
            deployEncoder = deployMotor.getEncoder();
            deployEncoder.setPosition(0);

            deployPID = new PIDController(IntakeK.deployP, IntakeK.deployI, IntakeK.deployD);
        }

        if (indexerMotor != null) {
            indexerMotor.setIdleMode(IdleMode.kCoast);
            indexerMotor.setInverted(true);
            indexerMotor.setSmartCurrentLimit(30); // so as to not kill the baby neo
            indexerMotor.setOpenLoopRampRate(0.1); // so as not to kill the baby neo
        }

        if (intakeMotor != null) {
            intakeMotor.setIdleMode(IdleMode.kCoast);
            intakeMotor.setInverted(true);
            intakeMotor.setOpenLoopRampRate(0.1); // so as not to destroy the belts
        }

        deployedLimitSwitch = new DigitalInput(2);
        retractedLimitSwitch = new DigitalInput(3);
        

        log = DataLogManager.getLog();
        posLog = new DoubleLogEntry(log, "/intake/pos");
        outputLog = new DoubleLogEntry(log, "/intake/output");
        setpointLog = new DoubleLogEntry(log, "/intake/setpoint");
        
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
            intakeMotor.set(power);
        }
    }

    /**
     * sets the indexer motor to the given power
     * but doesn't move if a ball is at the top
     * @param power the power to set the indexer to
     */
    public void setIndexer(double power) {
        if (indexerMotor != null) {
            indexerMotor.set(power);
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
        // if (deployMotor != null) {
            if (extended) { // if we're going to deploy it
                deployPID.setSetpoint(IntakeK.EXTENDED); // set the PID to deploy
                setpointLog.append(IntakeK.EXTENDED);
                direction = 1; // set the direction we're going in (for limit switch purposes)
            } else {
                deployPID.setSetpoint(IntakeK.RAISED); // same thing
                setpointLog.append(IntakeK.RAISED);
                direction = -1;
            }
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
        posLog.append(deployEncoder.getPosition());
        outputLog.append(deployMotor.get());
        
            
        if (direction != 0) { // don't start moving unless the code has started and the intake has been told to move,
                            // so it can be moved when powered on but disabled
            boolean deployed = !deployedLimitSwitch.get();
            if (deployed) { // if limit switch is pressed
                deployEncoder.setPosition(IntakeK.EXTENDED); // set the intake encoder to the correct position
            }
        
            if (direction == 1 && deployed) { // if the limit switch is pressed
                deployMotor.set(0); // stop the intake
            } else { // otherwise we're good to keep moving
                double power = deployPID.calculate(deployEncoder.getPosition()); // calculate the power
                if (Math.abs(power) > 0.8) { // limit the power to a max of 0.8             
                    power = Math.copySign(0.8, power);
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
                deployPID.setP(intakeP.getDouble(IntakeK.deployP));
                deployPID.setI(intakeI.getDouble(IntakeK.deployI));
                deployPID.setD(intakeD.getDouble(IntakeK.deployD));
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
        }
    }
}