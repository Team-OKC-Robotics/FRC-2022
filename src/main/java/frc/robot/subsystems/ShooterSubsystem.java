package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
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
    private NetworkTableEntry writeMode = tab.addPersistent("Write Mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.addPersistent("shooter ticks", 0).getEntry();
    private NetworkTableEntry shooterRPM = tab.addPersistent("shooter RPM", 0).getEntry();
    private NetworkTableEntry velocityError = tab.addPersistent("velocity error", 0).getEntry();

    // PID
    private NetworkTableEntry shootP = tab.addPersistent("Shooter kP", 0).getEntry();
    private NetworkTableEntry shootI = tab.addPersistent("Shooter kI", 0).getEntry();
    private NetworkTableEntry shootD = tab.addPersistent("Shooter kD", 0).getEntry();
    private NetworkTableEntry shootF = tab.addPersistent("Shooter kF", 0).getEntry();

    // presets
    private NetworkTableEntry preset1 = tab.addPersistent("close launchpad preset", ShootK.preset1).getEntry();
    private NetworkTableEntry preset2 = tab.addPersistent("far launchpad preset", ShootK.preset2).getEntry();


    public ShooterSubsystem() {
        //TODO change port numbers these are temporary
        shooterMotor1 = new TalonFX(10);
        triggerMotor = new CANSparkMax(22, MotorType.kBrushless);

        shooterMotor1.configFactoryDefault();
        shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), 0, 200);
        shooterMotor1.config_kP(0, ShootK.shootP, 200);
        shooterMotor1.config_kI(0, ShootK.shootI, 200);
        shooterMotor1.config_kD(0, ShootK.shootD, 200);
        shooterMotor1.config_kF(0, ShootK.shootF, 200);
    }

    public void setShooter(double RPM) {
        shooterMotor1.set(ControlMode.Velocity, RPM * 2048.0 / 600.0); // have to convert to units / 100ms or somethingS
    }

    public void setShooterPreset(ShooterPresets preset) {
        if (preset == ShooterPresets.CLOSE_LAUNCHPAD) {
            setShooter(preset1.getDouble(ShootK.preset1));
        } else if (preset == ShooterPresets.FAR_LAUNCHPAD) {
            setShooter(preset2.getDouble(ShootK.preset2));
        }
    }

    public void setTrigger(double power) {
        triggerMotor.set(power);
    }

    public boolean atShooterSetpoint() {
        return Math.abs(shooterMotor1.getClosedLoopError()) < 100;
    }

    @Override
    public void periodic() {
        // update Shuffelboard values
        ticks.setDouble(shooterMotor1.getSelectedSensorPosition());
        shooterRPM.setDouble(shooterMotor1.getSelectedSensorVelocity());
        velocityError.setDouble(shooterMotor1.getSelectedSensorVelocity());
        
        // Shuffleboard on-the-fly tuning
        if (writeMode.getBoolean(false)) {
            shooterMotor1.config_kP(0, shootP.getDouble(ShootK.shootP));
            shooterMotor1.config_kI(0, shootI.getDouble(ShootK.shootI));
            shooterMotor1.config_kD(0, shootD.getDouble(ShootK.shootD));
            shooterMotor1.config_kF(0, shootF.getDouble(ShootK.shootF));
        }
    }
}