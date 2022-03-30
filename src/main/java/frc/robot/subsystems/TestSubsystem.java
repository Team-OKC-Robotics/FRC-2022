package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
    // actuators
    private CANSparkMax testMotor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pid;
       
    public TestSubsystem() {
        // motor configuration
        testMotor = new CANSparkMax(1, MotorType.kBrushless);
        encoder = testMotor.getEncoder();
        pid = testMotor.getPIDController();
        resetEncoders();
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public void setPower(double power) {
        testMotor.set(power);
    }

    public void setPowerPID(double power) {
        pid.setReference(power, ControlType.kDutyCycle);
    }

    public setPID(double value) {
        pid.setReference(value, ControlType.kPosition);        
    }

    @Override
    public void periodic() {
    }
}
