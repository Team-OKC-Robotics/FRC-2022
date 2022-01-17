package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonFX rightExtendMotor;
    private CANSparkMax rightTiltMotor;
    private RelativeEncoder rightTiltEncoder;

    private WPI_TalonFX leftExtendMotor;
    private CANSparkMax leftTiltMotor;
    private RelativeEncoder leftTiltEncoder;

    private PIDController leftPID;
    private PIDController rightPID;

    //TODO shuffleboard
    
    public ClimberSubsystem() {
        //TODO change port numbers
        rightExtendMotor = new WPI_TalonFX(14);
        rightTiltMotor = new CANSparkMax(15, MotorType.kBrushless);
        rightExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        rightTiltEncoder = rightTiltMotor.getEncoder();
        
        leftExtendMotor = new WPI_TalonFX(16);
        leftTiltMotor = new CANSparkMax(17, MotorType.kBrushless);
        leftExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        leftTiltEncoder = leftTiltMotor.getEncoder();
        
        // call this method to get the sensor position leftExtendMotor.setSelectedSensorPosition(0);

        leftPID = new PIDController(ClimbK.leftP, ClimbK.leftI, ClimbK.leftD);
        rightPID = new PIDController(ClimbK.rightP, ClimbK.rightI, ClimbK.rightD);

        //TODO configure motors and stuff
    }

    public void setLeft() {
        
    }

    public void setRight() {

    }
}
