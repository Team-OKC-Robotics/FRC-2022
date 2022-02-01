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
    /**
     * An enum to account for all the different preset values we might want to shoot from
     * this should be eventually (hopefully) rendered obsolete by just using vision and good maths
     */
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
    private NetworkTableEntry shootP = tab.addPersistent("Shooter kP", ShootK.shootP).getEntry();
    private NetworkTableEntry shootI = tab.addPersistent("Shooter kI", ShootK.shootI).getEntry();
    private NetworkTableEntry shootD = tab.addPersistent("Shooter kD", ShootK.shootD).getEntry();
    private NetworkTableEntry shootF = tab.addPersistent("Shooter kF", ShootK.shootF).getEntry();

    // presets
    private NetworkTableEntry preset1 = tab.addPersistent("close launchpad preset", ShootK.preset1).getEntry();
    private NetworkTableEntry preset2 = tab.addPersistent("far launchpad preset", ShootK.preset2).getEntry();

    /**
     * Makes a new ShooterSubsystem
     * the shooter controls the shooter motor(s?) and the "trigger motor"
     */
    public ShooterSubsystem() {
        //TODO change port numbers these are temporary
        shooterMotor1 = new TalonFX(21);
        triggerMotor = new CANSparkMax(22, MotorType.kBrushless); //TODO configure this motor

        if (shooterMotor1 != null) {
            shooterMotor1.configFactoryDefault();
            shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), 0, 200);
            shooterMotor1.config_kP(0, ShootK.shootP, 200);
            shooterMotor1.config_kI(0, ShootK.shootI, 200);
            shooterMotor1.config_kD(0, ShootK.shootD, 200);
            shooterMotor1.config_kF(0, ShootK.shootF, 200);
        }
    }

    /**
     * sets the shooter to PID to the given velocity
     * @param RPM the rpm to set the shooter to
     */
    public void setShooter(double RPM) {
        if (shooterMotor1 != null) {
            shooterMotor1.set(ControlMode.Velocity, RPM * 2048.0 / 600.0); // have to convert to units / 100ms or somethingS
        }
    }

    /**
     * sets the shooter to the given preset value
     * @param preset the preset to set the shooter to
     */
    public void setShooterPreset(ShooterPresets preset) {
        if (preset == ShooterPresets.CLOSE_LAUNCHPAD) {
            setShooter(preset1.getDouble(ShootK.preset1));
        } else if (preset == ShooterPresets.FAR_LAUNCHPAD) {
            setShooter(preset2.getDouble(ShootK.preset2));
        }
    }

    /**
     * sets the trigger to the given power
     * @param power the power to set the trigger motor to
     */
    public void setTrigger(double power) {
        if (triggerMotor != null) {
            triggerMotor.set(power);
        }
    }

    /**
     * returns if the shooter PID is at the last set setpoint or not
     * @return if the shooter is at its setpoint
     */
    public boolean atShooterSetpoint() {
        if (shooterMotor1 != null) {
            //TODO check and make sure I'm accounting for velocity error correctly
            return Math.abs(shooterMotor1.getClosedLoopError()) < 100 && shooterMotor1.getErrorDerivative() < 100; //???
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
            if (shooterMotor1 != null) {
                shooterMotor1.config_kP(0, shootP.getDouble(ShootK.shootP));
                shooterMotor1.config_kI(0, shootI.getDouble(ShootK.shootI));
                shooterMotor1.config_kD(0, shootD.getDouble(ShootK.shootD));
                shooterMotor1.config_kF(0, shootF.getDouble(ShootK.shootF));
            }
        }
    }
}