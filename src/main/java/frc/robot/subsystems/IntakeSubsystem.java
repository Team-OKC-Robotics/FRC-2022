package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax deployMotor;
    //private CANSparkMax intakeMotor;
    private CANSparkMax indexerMotor;

    //FIXME temporary motor because we don't have the NEOs wired up/mounted yet
    private PWMSparkMax intakeMotor;

    private boolean extended = false;
    private RelativeEncoder deployEncoder;
    private PIDController deployPID;
    private SparkMaxPIDController extendPID;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("intake");
    private NetworkTableEntry writeMode = tab.add("write mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.addPersistent("intake ticks", 0).getEntry();
    private NetworkTableEntry velocity = tab.addPersistent("intake velocity", 0).getEntry();
    
    // PID
    private NetworkTableEntry intakeP = tab.addPersistent("Intake kP", 0).getEntry();
    private NetworkTableEntry intakeI = tab.addPersistent("Intake kI", 0).getEntry();
    private NetworkTableEntry intakeD = tab.addPersistent("Intake kD", 0).getEntry();
    

    // I don't think there needs to be any shuffleboard stuff here
    // we could do some weird stuff with like hasBall() but that's not important right now
    
    public IntakeSubsystem() {
        //TODO change id numbers
        deployMotor = new CANSparkMax(10, MotorType.kBrushless);
        //intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(12, MotorType.kBrushless);
        
        //TEMP TEMP TEMP
        intakeMotor = new PWMSparkMax(1);

        deployPID = new PIDController(IntakeK.deployP, IntakeK.deployI, IntakeK.deployD);
        extendPID = deployMotor.getPIDController(); //TODO configure this because it's gonna not work right because going down is gonna kill stuff
        deployEncoder = deployMotor.getEncoder();
    }

    public void setIntake(double power) {
        intakeMotor.set(power);
    }

    public void setIndexer(double power) {
        indexerMotor.set(power);
    }

    public void setExtended(boolean extended) {
        if (this.extended != extended) {
            if (extended) {
                extendPID.setReference(IntakeK.EXTENDED, ControlType.kPosition);
            } else {
                extendPID.setReference(IntakeK.RAISED, ControlType.kPosition);
            }
        }
        this.extended = extended;
    }

    public boolean isExtended() {
        return extended;
    }

    @Override
    public void periodic() {
        ticks.setDouble(deployEncoder.getPosition());
        velocity.setDouble(deployEncoder.getVelocity());

        if (writeMode.getBoolean(false)) {
            extendPID.setP(intakeP.getDouble(IntakeK.deployP));
            extendPID.setI(intakeI.getDouble(IntakeK.deployI));
            extendPID.setD(intakeD.getDouble(IntakeK.deployD));
        }
    }
}