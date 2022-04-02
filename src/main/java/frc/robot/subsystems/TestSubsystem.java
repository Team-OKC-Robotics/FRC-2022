package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
    // actuators
    private CANSparkMax testMotor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pid;
    private double setpoint = 0;
    private boolean stopped = false;
       
    public TestSubsystem() {
        // motor configuration
        testMotor = new CANSparkMax(8, MotorType.kBrushless);
        encoder = testMotor.getEncoder();
        pid = testMotor.getPIDController();
        pid.setP(0.01);
        resetEncoders();
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public void setPower(double power) {
        testMotor.set(power);
        stopped = false;
    }

    public void setPowerPID(double power) {
        pid.setReference(power, ControlType.kDutyCycle);
    }

    public void setPID(double value) {
        setpoint = value;
        pid.setReference(value, ControlType.kPosition);     
    }

    public boolean atSetpoint() {
        return encoder.getPosition() == setpoint;
    }

    public void finish() {
        pid.setReference(setpoint, ControlType.kPosition);
    }

    public void manualStop() {
        stopped = true;
        setpoint = encoder.getPosition();
    }

    @Override
    public void periodic() {
        if (stopped) {
            pid.setReference(setpoint, ControlType.kPosition);
        }
    }
}
