package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShootK;

public class ShooterSubsystem extends SubsystemBase {
    /**
     * An enum to account for all the different preset values we might want to shoot from
     * this should be eventually (hopefully) rendered obsolete by just using vision and good maths
     */
    public enum ShooterPresets {
        CLOSE_LAUNCHPAD,
        FAR_LAUNCHPAD,
        TARMAC_LINE,
        CENTER_LINE,
        LOW_GOAL
    }

    // actuators
    private TalonFX shooterMotor1;
    private PIDController shooterPID;
    private CANSparkMax triggerMotor; // the shooter tower

    // sensors
    private DigitalInput ballDetector;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("shooter");
    private NetworkTableEntry writeMode = tab.add("Write Mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.add("shooter ticks", 0).getEntry();
    private NetworkTableEntry shooterRPM = tab.add("shooter RPM", 0).getEntry();
    private NetworkTableEntry shooterOutput = tab.add("shooter output", 0).getEntry();
    private NetworkTableEntry velocityError = tab.add("velocity error", 0).getEntry();
    private NetworkTableEntry hasBall = tab.add("has ball?", false).getEntry();

    // PID
    private NetworkTableEntry shootP = tab.add("Shooter kP", ShootK.shootP).getEntry();
    private NetworkTableEntry shootI = tab.add("Shooter kI", ShootK.shootI).getEntry();
    private NetworkTableEntry shootD = tab.add("Shooter kD", ShootK.shootD).getEntry();
    private NetworkTableEntry shootF = tab.add("Shooter kF", ShootK.shootF).getEntry();
    private NetworkTableEntry shooterGood = tab.add("shooter good", false).getEntry();

    // presets
    private NetworkTableEntry preset1 = tab.add("close launchpad preset", ShootK.preset1).getEntry();
    private NetworkTableEntry preset2 = tab.add("far launchpad preset", ShootK.preset2).getEntry();
    private NetworkTableEntry preset3 = tab.add("tarmac line preset", ShootK.tarmacPreset).getEntry();
    private NetworkTableEntry preset4 = tab.add("low goal preset", ShootK.lowGoalPreset).getEntry();
    
    /**
     * Makes a new ShooterSubsystem
     * the shooter controls the shooter motor(s?) and the "trigger motor"
     */
    public ShooterSubsystem() {
        shooterMotor1 = new TalonFX(8);

        if (shooterMotor1 != null) {
            shooterMotor1.configFactoryDefault();
            shooterMotor1.setInverted(InvertType.InvertMotorOutput);
            shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(), 0, 200);
            shooterMotor1.configNeutralDeadband(0);
            shooterMotor1.setNeutralMode(NeutralMode.Coast);
            shooterMotor1.config_kP(0, ShootK.shootP, 200);
            shooterMotor1.config_kI(0, ShootK.shootI, 200);
            shooterMotor1.config_kD(0, ShootK.shootD, 200);
            // shooterMotor1.config_kF(0, ShootK.shootF, 200);

            shooterMotor1.setSelectedSensorPosition(0);
            shooterPID = new PIDController(ShootK.shootP, ShootK.shootI, ShootK.shootD);
        }

        triggerMotor = new CANSparkMax(9, MotorType.kBrushless);
        triggerMotor.setIdleMode(IdleMode.kCoast);
        ballDetector = new DigitalInput(9);
    }

    /**
     * sets the shooter to PID to the given velocity
     * @param RPM the rpm to set the shooter to
     */
    public void setShooter(double RPM) {
        if (shooterMotor1 != null) {
            // shooterMotor1.set(ControlMode.PercentOutput, -shooterPID.calculate(RPM, shooterMotor1.getSelectedSensorVelocity()));
            shooterMotor1.set(ControlMode.Velocity, RPM/* * 2048.0 / 600.0*/, DemandType.ArbitraryFeedForward, 0.4); // have to convert to units / 100ms or something?
        }
    }

    public void stopShooter() {
        shooterMotor1.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * sets the shooter to the given preset value
     * @param preset the preset to set the shooter to
     */
    public void setShooterPreset(ShooterPresets preset) {
        if (preset == ShooterPresets.LOW_GOAL) {
            setShooter(preset4.getDouble(ShootK.lowGoalPreset));
        } else if (preset == ShooterPresets.TARMAC_LINE) {
            setShooter(preset3.getDouble(ShootK.tarmacPreset));
        } else if (preset == ShooterPresets.CLOSE_LAUNCHPAD) {
            setShooter(preset2.getDouble(ShootK.preset2));
        } else if (preset == ShooterPresets.FAR_LAUNCHPAD) {
            setShooter(preset1.getDouble(ShootK.preset1));
        }
    }

    /**
     * returns if the shooter PID is at the last set setpoint or not
     * @return if the shooter is at its setpoint
     */
    public boolean atShooterSetpoint() {
        if (shooterMotor1 != null) {
            // return shooterPID.atSetpoint();
            // return Math.abs(shooterMotor1.getSelectedSensorVelocity() - ShootK.tarmacPreset) < 500 && shooterPID.getVelocityError() < 100; //??? I know that's the only preset we're going for but 
            //TODO check and make sure I'm accounting for velocity error correctly
            return Math.abs(shooterMotor1.getClosedLoopError()) < 100 && shooterMotor1.getErrorDerivative() < 10; //???
        }
        return true;
    }

    // sets the shooter tower ("trigger") motor with ball detection
    public void setTrigger(double power) {
        if (triggerMotor != null) {
            if (!ballDetector.get()) { // ball detector is inverse logic, so if we have ball
                if (power <= 0) { // don't let the ball move forwards
                    triggerMotor.set(power);
                } else {
                    triggerMotor.set(0);
                }
            } else {
                triggerMotor.set(power); // otherwise run as much as you want
            }
        }
    }

    // ignores ball detection
    public void feed(double power) {
        if (triggerMotor != null) {
            triggerMotor.set(power);
        }
    }

    @Override
    public void periodic() {
        if (!Constants.competition) {
            hasBall.setBoolean(ballDetector.get());

            // update Shuffelboard values
            if (shooterMotor1 != null) {
                ticks.setDouble(shooterMotor1.getSelectedSensorPosition());
                shooterRPM.setDouble(shooterMotor1.getSelectedSensorVelocity());
                velocityError.setDouble(shooterMotor1.getClosedLoopError());
                shooterGood.setBoolean(atShooterSetpoint());
                shooterOutput.setDouble(shooterMotor1.getMotorOutputPercent());
            }
            
            // Shuffleboard on-the-fly tuning
            if (writeMode.getBoolean(false)) {
                if (shooterMotor1 != null) {
                    // shooterMotor1.config_kP(0, shootP.getDouble(ShootK.shootP));
                    // shooterMotor1.config_kI(0, shootI.getDouble(ShootK.shootI));
                    // shooterMotor1.config_kD(0, shootD.getDouble(ShootK.shootD));
                    // shooterMotor1.config_kF(0, shootF.getDouble(ShootK.shootF));
                    // shooterPID.setP(shootP.getDouble(ShootK.shootP));
                    // shooterPID.setI(shootI.getDouble(ShootK.shootI));
                    // shooterPID.setD(shootD.getDouble(ShootK.shootD));
                }
            }
        }
    }
}