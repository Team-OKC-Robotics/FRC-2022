package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonFX rightExtendMotor;
    private CANSparkMax rightTiltMotor;
    private WPI_TalonFX leftExtendMotor;
    private CANSparkMax leftTiltMotor;
    
    //TODO figure out what functionality the climber actually has and how it works because I don't know
    public ClimberSubsystem() {
        //TODO change port numbers
        rightExtendMotor = new WPI_TalonFX(14);
        rightTiltMotor = new CANSparkMax(15, MotorType.kBrushless);
        leftExtendMotor = new WPI_TalonFX(16);
        leftTiltMotor = new CANSparkMax(17, MotorType.kBrushless);

        //TODO configure motors and stuff
    }

    public void climb() {
        //??????? how the heck does this subsystem work?????
    }

    public void extend() {
        
    }
}
