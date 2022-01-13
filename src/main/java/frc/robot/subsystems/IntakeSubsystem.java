package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    //TODO change actuator type and number this is just me guessing
    private CANSparkMax deployMotor;
    private CANSparkMax intakeMotor;

    // I don't think there needs to be any shuffleboard stuff here
    
    public IntakeSubsystem() {
        //TODO change id numbers
        deployMotor = new CANSparkMax(10, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
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