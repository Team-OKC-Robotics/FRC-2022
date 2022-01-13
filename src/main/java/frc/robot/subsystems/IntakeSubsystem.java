package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmK;

public class IntakeSubsystem extends SubsystemBase {
    //TODO change actuator type and number this is just me guessing
    private CANSparkMax deployMotor;
    private CANSparkMax intakeMotor;

    // I don't think there needs to be any shuffleboard stuff here
    
    public IntakeSubsystem() {
        //TODO change id numbers
        deployMotor = new CANSparkMax(MotorType.kBrushless, 10);
        intakeMotor = new CANSparkMax(MotorType.kBrushless, 11);
    }

    public void set(double power) {
        intakeMotor.set(power);
    }

    public void setExtended(boolean extended) {
        //TODO logic here
        // idk if we even want this method like this having two separate methods (like extend() and retract()) might be better
    }

    @Override
    public void periodic() {
        
    }
}