package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax deployMotor;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexerMotor;

    private boolean extended = false;
    private RelativeEncoder deployEncoder;
    private SparkMaxPIDController extendPID;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("intake");
    private NetworkTableEntry writeMode = tab.add("write mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.add("intake ticks", 0).getEntry();
    private NetworkTableEntry velocity = tab.add("intake velocity", 0).getEntry();
    
    // PID
    private NetworkTableEntry intakeP = tab.add("Intake kP", IntakeK.deployP).getEntry();
    private NetworkTableEntry intakeI = tab.add("Intake kI", IntakeK.deployI).getEntry();
    private NetworkTableEntry intakeD = tab.add("Intake kD", IntakeK.deployD).getEntry();
    
    /**
     * makes a new IntakeSubsystem
     * the intake consists of the intake itself, which goes up and down, and the powered wheels to suck balls in
     * there's also a 'pass-through' or 'indexer' motor to move the balls along to the shooter
     */
    public IntakeSubsystem() {
        //TODO change id numbers
        deployMotor = new CANSparkMax(10, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(12, MotorType.kBrushless);
        
        if (deployMotor != null) {
            extendPID = deployMotor.getPIDController(); //TODO configure this because it's gonna not work right because going down is gonna kill stuff
            deployEncoder = deployMotor.getEncoder();
        }
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
     * @param power the power to set the indexer to
     */
    public void setIndexer(double power) {
        if (indexerMotor != null) {
            indexerMotor.set(power);
        }
    }

    /**
     * set the intake to be extended/deployed
     * @param extended if the intake should be extended/deployed or not
     */
    public void setExtended(boolean extended) {
        if (this.extended != extended) {
            if (extendPID != null) {
                if (extended) {
                    extendPID.setReference(IntakeK.EXTENDED, ControlType.kPosition);
                } else {
                    extendPID.setReference(IntakeK.RAISED, ControlType.kPosition);
                }
            }
        }
        this.extended = extended;
    }

    /**
     * returns if the intake is extended/deployed
     * @return true if the intake is extended
     */
    public boolean isExtended() {
        return extended;
    }

    @Override
    public void periodic() {
        if (deployEncoder != null) {
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