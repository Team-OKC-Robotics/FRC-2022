package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonFX rightExtendMotor;
    private CANSparkMax rightTiltMotor;
    private RelativeEncoder rightTiltEncoder;

    private WPI_TalonFX leftExtendMotor;
    private CANSparkMax leftTiltMotor;
    private RelativeEncoder leftTiltEncoder;

    private SparkMaxPIDController leftPID;
    private SparkMaxPIDController rightPID;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("climber");
    private NetworkTableEntry writeMode = tab.add("Write Mode", false).getEntry();

    // sensor
    private NetworkTableEntry leftTiltPos = tab.add("Left Tilt Position", 0).getEntry();
    private NetworkTableEntry leftExtendPos = tab.add("Left Extend Position", 0).getEntry();

    private NetworkTableEntry rightTiltPos = tab.add("Right Tilt Position", 0).getEntry();
    private NetworkTableEntry rightExtendPos = tab.add("Right Extend Position", 0).getEntry();

    // PIDs (boi there's a lot)
    private NetworkTableEntry leftExtendP = tab.add("Left Extend kP", ClimbK.leftExtendP).getEntry();
    private NetworkTableEntry leftExtendI = tab.add("Left Extend kI", ClimbK.leftExtendI).getEntry();
    private NetworkTableEntry leftExtendD = tab.add("Left Extend kD", ClimbK.leftExtendD).getEntry();
    private NetworkTableEntry leftExtendF = tab.add("Left Extend kF", ClimbK.leftExtendF).getEntry();

    private NetworkTableEntry leftTiltP = tab.add("Left Tilt kP", ClimbK.leftTiltP).getEntry();
    private NetworkTableEntry leftTiltI = tab.add("Left Tilt kI", ClimbK.leftTiltI).getEntry();
    private NetworkTableEntry leftTiltD = tab.add("Left Tilt kD", ClimbK.leftTiltD).getEntry();
    
    private NetworkTableEntry rightExtendP = tab.add("Right Extend kP", ClimbK.rightExtendP).getEntry();
    private NetworkTableEntry rightExtendI = tab.add("Right Extend kI", ClimbK.rightExtendI).getEntry();
    private NetworkTableEntry rightExtendD = tab.add("Right Extend kD", ClimbK.rightExtendD).getEntry();
    private NetworkTableEntry rightExtendF = tab.add("Right Extend kF", ClimbK.rightExtendF).getEntry();

    private NetworkTableEntry rightTiltP = tab.add("Right Tilt kP", ClimbK.rightTiltP).getEntry();
    private NetworkTableEntry rightTiltI = tab.add("Right Tilt kI", ClimbK.rightTiltI).getEntry();
    private NetworkTableEntry rightTiltD = tab.add("Right Tilt kD", ClimbK.rightTiltD).getEntry();
    
    public ClimberSubsystem() {
        //TODO change port numbers
        rightExtendMotor = new WPI_TalonFX(14);
        rightTiltMotor = new CANSparkMax(15, MotorType.kBrushless);
        rightExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        rightTiltEncoder = rightTiltMotor.getEncoder();
        
        leftExtendMotor = new WPI_TalonFX(16);
        leftTiltMotor = new CANSparkMax(17, MotorType.kBrushless);
        // call this method to get the sensor position leftExtendMotor.setSelectedSensorPosition(0);
        leftExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        leftTiltEncoder = leftTiltMotor.getEncoder();
        
        leftPID = leftTiltMotor.getPIDController();
        rightPID = rightTiltMotor.getPIDController();

        //TODO configure the falcon 500s and their PIDs and stuff
    }

    public void setLeftExtend(double inches) {
        leftExtendMotor.set(TalonFXControlMode.Position, inches);
    }

    public void setLeftTilt(double angle) {
        leftPID.setReference(angle, ControlType.kPosition);
    }

    public void setRightExtend(double inches) {
        rightExtendMotor.set(TalonFXControlMode.Position, inches);
    }

    public void setRightTilt(double angle) {
        rightPID.setReference(angle, ControlType.kPosition);
    }

    public boolean atLeftExtendSetpoint() {
        return Math.abs(leftExtendMotor.getClosedLoopError()) < 0.5; //TODO change the tolerance
    }

    public boolean atRightExtendSetpoint() {
        return Math.abs(rightExtendMotor.getClosedLoopError()) < 0.5; //TODO change the tolerance
    }

    public boolean atLeftTiltSetpoint() {
        //BUG FIXME
        
        return false; // I am at a loss as to how you know if the integrated closed-loop control is anywhere near its target or not
        //return leftPID.atSetpoint(); //TODO change the tolerance
    }

    public boolean atRightTiltSetpoint() {
        return false; //FIXME
    }

    @Override
    public void periodic() {
        leftTiltPos.setDouble(leftTiltEncoder.getPosition());
        leftExtendPos.setDouble(leftExtendMotor.getSelectedSensorPosition());

        rightTiltPos.setDouble(rightTiltEncoder.getVelocity());
        rightExtendPos.setDouble(rightExtendMotor.getSelectedSensorPosition());

        if (writeMode.getBoolean(false)) {
            leftPID.setP(leftTiltP.getDouble(ClimbK.leftTiltP));
            leftPID.setI(leftTiltI.getDouble(ClimbK.leftTiltI));
            leftPID.setD(leftTiltD.getDouble(ClimbK.leftTiltD));

            leftExtendMotor.config_kP(0, leftExtendP.getDouble(ClimbK.leftExtendP));
            leftExtendMotor.config_kI(0, leftExtendI.getDouble(ClimbK.leftExtendI));
            leftExtendMotor.config_kD(0, leftExtendD.getDouble(ClimbK.leftExtendD));
            leftExtendMotor.config_kF(0, leftExtendF.getDouble(ClimbK.leftExtendF));
            
            rightPID.setP(rightTiltP.getDouble(ClimbK.rightTiltP));
            rightPID.setI(rightTiltI.getDouble(ClimbK.rightTiltI));
            rightPID.setD(rightTiltD.getDouble(ClimbK.rightTiltD));

            rightExtendMotor.config_kP(0, rightExtendP.getDouble(ClimbK.rightExtendP));
            rightExtendMotor.config_kI(0, rightExtendI.getDouble(ClimbK.rightExtendI));
            rightExtendMotor.config_kD(0, rightExtendD.getDouble(ClimbK.rightExtendD));
            rightExtendMotor.config_kF(0, rightExtendF.getDouble(ClimbK.rightExtendF));
        }
    }
}
