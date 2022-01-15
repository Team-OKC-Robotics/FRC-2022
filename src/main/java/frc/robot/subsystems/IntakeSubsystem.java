package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax deployMotor;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexerMotor;

    // I don't think there needs to be any shuffleboard stuff here
    // we could do some weird stuff with like hasBall() but that's not important right now
    
    public IntakeSubsystem() {
        //TODO change id numbers
        deployMotor = new CANSparkMax(10, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(12, MotorType.kBrushless);
    }

    public void setIntake(double power) {
        intakeMotor.set(power);
    }

    public void setIndexer(double power) {
        indexerMotor.set(power);
    }

    public void setExtended(boolean extended) {
        //TODO logic here
        // idk if we even want this method like this having two separate methods (like extend() and retract()) might be better
    }

    @Override
    public void periodic() {
        // idk if we need to do anything here
    }
}