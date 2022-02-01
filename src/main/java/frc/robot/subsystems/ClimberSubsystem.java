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
    private NetworkTableEntry writeMode = tab.addPersistent("Write Mode", false).getEntry();

    // sensor
    private NetworkTableEntry leftTiltPos = tab.addPersistent("Left Tilt Position", 0).getEntry();
    private NetworkTableEntry leftExtendPos = tab.addPersistent("Left Extend Position", 0).getEntry();

    private NetworkTableEntry rightTiltPos = tab.addPersistent("Right Tilt Position", 0).getEntry();
    private NetworkTableEntry rightExtendPos = tab.addPersistent("Right Extend Position", 0).getEntry();

    // PIDs (boi there's a lot)
    private NetworkTableEntry leftExtendP = tab.addPersistent("Left Extend kP", ClimbK.leftExtendP).getEntry();
    private NetworkTableEntry leftExtendI = tab.addPersistent("Left Extend kI", ClimbK.leftExtendI).getEntry();
    private NetworkTableEntry leftExtendD = tab.addPersistent("Left Extend kD", ClimbK.leftExtendD).getEntry();
    private NetworkTableEntry leftExtendF = tab.addPersistent("Left Extend kF", ClimbK.leftExtendF).getEntry();

    private NetworkTableEntry leftTiltP = tab.addPersistent("Left Tilt kP", ClimbK.leftTiltP).getEntry();
    private NetworkTableEntry leftTiltI = tab.addPersistent("Left Tilt kI", ClimbK.leftTiltI).getEntry();
    private NetworkTableEntry leftTiltD = tab.addPersistent("Left Tilt kD", ClimbK.leftTiltD).getEntry();
    
    private NetworkTableEntry rightExtendP = tab.addPersistent("Right Extend kP", ClimbK.rightExtendP).getEntry();
    private NetworkTableEntry rightExtendI = tab.addPersistent("Right Extend kI", ClimbK.rightExtendI).getEntry();
    private NetworkTableEntry rightExtendD = tab.addPersistent("Right Extend kD", ClimbK.rightExtendD).getEntry();
    private NetworkTableEntry rightExtendF = tab.addPersistent("Right Extend kF", ClimbK.rightExtendF).getEntry();

    private NetworkTableEntry rightTiltP = tab.addPersistent("Right Tilt kP", ClimbK.rightTiltP).getEntry();
    private NetworkTableEntry rightTiltI = tab.addPersistent("Right Tilt kI", ClimbK.rightTiltI).getEntry();
    private NetworkTableEntry rightTiltD = tab.addPersistent("Right Tilt kD", ClimbK.rightTiltD).getEntry();
    
    public ClimberSubsystem() {
        //TODO change port numbers
        rightExtendMotor = new WPI_TalonFX(14);
        rightTiltMotor = new CANSparkMax(15, MotorType.kBrushless);

        // set up the right side
        if (rightExtendMotor != null) {
            rightExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        }

        if (rightTiltEncoder != null) {
            rightTiltEncoder = rightTiltMotor.getEncoder();
        }
        
        // set up the left side
        leftExtendMotor = new WPI_TalonFX(16);
        leftTiltMotor = new CANSparkMax(17, MotorType.kBrushless);
        
        if (leftExtendMotor != null) {
            leftExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        }

        if (leftTiltEncoder != null) {
            leftTiltEncoder = leftTiltMotor.getEncoder();
        }
        
        if (leftTiltMotor != null) {
            leftPID = leftTiltMotor.getPIDController();
        }

        if (rightTiltMotor != null) {
            rightPID = rightTiltMotor.getPIDController();
        }

        //TODO configure the falcon 500s and their PIDs and stuff
    }

    public void setLeftExtend(double inches) {
        if (leftExtendMotor != null) {
            leftExtendMotor.set(TalonFXControlMode.Position, inches);
        }
    }

    public void setLeftTilt(double angle) {
        if (leftPID != null) {
            leftPID.setReference(angle, ControlType.kPosition);
        }
    }

    public void setRightExtend(double inches) {
        if (rightExtendMotor != null) {
            rightExtendMotor.set(TalonFXControlMode.Position, inches);
        }
    }

    public void setRightTilt(double angle) {
        if (rightPID != null) {
            rightPID.setReference(angle, ControlType.kPosition);
        }
    }

    @Override
    public void periodic() {
        if (leftTiltEncoder != null) {
            leftTiltPos.setDouble(leftTiltEncoder.getPosition());
        }

        if (leftExtendMotor != null) {
            leftExtendPos.setDouble(leftExtendMotor.getSelectedSensorPosition());
        }

        if (rightTiltEncoder != null) {
            rightTiltPos.setDouble(rightTiltEncoder.getVelocity());
        }
        if (rightExtendMotor != null) {
            rightExtendPos.setDouble(rightExtendMotor.getSelectedSensorPosition());
        }


        if (writeMode.getBoolean(false)) {
            if (leftPID != null) {
                leftPID.setP(leftTiltP.getDouble(ClimbK.leftTiltP));
                leftPID.setI(leftTiltI.getDouble(ClimbK.leftTiltI));
                leftPID.setD(leftTiltD.getDouble(ClimbK.leftTiltD));
            }

            if (leftExtendMotor != null) {
                leftExtendMotor.config_kP(0, leftExtendP.getDouble(ClimbK.leftExtendP));
                leftExtendMotor.config_kI(0, leftExtendI.getDouble(ClimbK.leftExtendI));
                leftExtendMotor.config_kD(0, leftExtendD.getDouble(ClimbK.leftExtendD));
                leftExtendMotor.config_kF(0, leftExtendF.getDouble(ClimbK.leftExtendF));
            }

            if (rightPID != null) {
                rightPID.setP(rightTiltP.getDouble(ClimbK.rightTiltP));
                rightPID.setI(rightTiltI.getDouble(ClimbK.rightTiltI));
                rightPID.setD(rightTiltD.getDouble(ClimbK.rightTiltD));
            }

            if (rightExtendMotor != null) {
                rightExtendMotor.config_kP(0, rightExtendP.getDouble(ClimbK.rightExtendP));
                rightExtendMotor.config_kI(0, rightExtendI.getDouble(ClimbK.rightExtendI));
                rightExtendMotor.config_kD(0, rightExtendD.getDouble(ClimbK.rightExtendD));
                rightExtendMotor.config_kF(0, rightExtendF.getDouble(ClimbK.rightExtendF));
            }
        }
    }
}
