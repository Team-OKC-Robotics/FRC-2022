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
    private NetworkTableEntry ticks = tab.addPersistent("intake ticks", 0).getEntry();
    private NetworkTableEntry velocity = tab.addPersistent("intake velocity", 0).getEntry();
    
    // PID
    private NetworkTableEntry intakeP = tab.addPersistent("Intake kP", IntakeK.deployP).getEntry();
    private NetworkTableEntry intakeI = tab.addPersistent("Intake kI", IntakeK.deployI).getEntry();
    private NetworkTableEntry intakeD = tab.addPersistent("Intake kD", IntakeK.deployD).getEntry();
    

    // I don't think there needs to be any shuffleboard stuff here
    // we could do some weird stuff with like hasBall() but that's not important right now
    
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

    public void setIntake(double power) {
        if (intakeMotor != null) {
            intakeMotor.set(power);
        }
    }

    public void setIndexer(double power) {
        if (indexerMotor != null) {
            indexerMotor.set(power);
        }
    }

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
            extendPID.setP(intakeP.getDouble(IntakeK.deployP));
            extendPID.setI(intakeI.getDouble(IntakeK.deployI));
            extendPID.setD(intakeD.getDouble(IntakeK.deployD));
        }
    }
}