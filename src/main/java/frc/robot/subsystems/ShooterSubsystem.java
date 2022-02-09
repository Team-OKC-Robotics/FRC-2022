package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootK;

public class ShooterSubsystem extends SubsystemBase {

    private enum ShooterPresets {
        CLOSE_LAUNCHPAD,
        FAR_LAUNCHPAD,
        TARMAC_LINE,
        CENTER_LINE,
        LOW_GOAL
    }

    // actuators
    private TalonFX shooterMotor1;
    private CANSparkMax triggerMotor;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("shooter");
    private NetworkTableEntry writeMode = tab.add("Write Mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.add("shooter ticks", 0).getEntry();
    private NetworkTableEntry shooterRPM = tab.add("shooter RPM", 0).getEntry();
    private NetworkTableEntry velocityError = tab.add("velocity error", 0).getEntry();

    // PID
    private NetworkTableEntry shootP = tab.add("Shooter kP", ShootK.shootP).getEntry();
    private NetworkTableEntry shootI = tab.add("Shooter kI", ShootK.shootI).getEntry();
    private NetworkTableEntry shootD = tab.add("Shooter kD", ShootK.shootD).getEntry();
    private NetworkTableEntry shootF = tab.add("Shooter kF", ShootK.shootF).getEntry();

    // presets
    private NetworkTableEntry preset1 = tab.add("close launchpad preset", ShootK.preset1).getEntry();
    private NetworkTableEntry preset2 = tab.add("far launchpad preset", ShootK.preset2).getEntry();


    public ShooterSubsystem() {
        //TODO change port numbers these are temporary
        shooterMotor1 = new TalonFX(21);
        triggerMotor = new CANSparkMax(22, MotorType.kBrushless);

        shooterMotor1.configFactoryDefault();
        shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), 0, 200);
        shooterMotor1.config_kP(0, ShootK.shootP, 200);
        shooterMotor1.config_kI(0, ShootK.shootI, 200);
        shooterMotor1.config_kD(0, ShootK.shootD, 200);
        shooterMotor1.config_kF(0, ShootK.shootF, 200);

        triggerMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setShooter(double RPM) {
        if (shooterMotor1 != null) {
            shooterMotor1.set(ControlMode.Velocity, RPM * 2048.0 / 600.0); // have to convert to units / 100ms or somethingS
        }
    }

    public void setShooterPreset(ShooterPresets preset) {
        if (preset == ShooterPresets.CLOSE_LAUNCHPAD) {
            setShooter(preset1.getDouble(ShootK.preset1));
        } else if (preset == ShooterPresets.FAR_LAUNCHPAD) {
            setShooter(preset2.getDouble(ShootK.preset2));
        }
    }

    public void setTrigger(double power) {
        if (triggerMotor != null) {
            triggerMotor.set(power);
        }
    }

    public boolean atShooterSetpoint() {
        if (shooterMotor1 != null) {
            return Math.abs(shooterMotor1.getClosedLoopError()) < 100;
        }
        return true;
    }

    @Override
    public void periodic() {
        // update Shuffelboard values
        if (shooterMotor1 != null) {
            ticks.setDouble(shooterMotor1.getSelectedSensorPosition());
            shooterRPM.setDouble(shooterMotor1.getSelectedSensorVelocity());
            velocityError.setDouble(shooterMotor1.getSelectedSensorVelocity());
        }
        
        // Shuffleboard on-the-fly tuning
        if (writeMode.getBoolean(false)) {
            shooterMotor1.config_kP(0, shootP.getDouble(ShootK.shootP));
            shooterMotor1.config_kI(0, shootI.getDouble(ShootK.shootI));
            shooterMotor1.config_kD(0, shootD.getDouble(ShootK.shootD));
            shooterMotor1.config_kF(0, shootF.getDouble(ShootK.shootF));
        }
    }
}