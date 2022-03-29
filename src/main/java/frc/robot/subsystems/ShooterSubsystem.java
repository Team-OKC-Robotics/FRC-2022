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
        NORMAL_SHOT,
        AGAINST_HUB,
        LOW_GOAL,
        FAR_SHOT
    }

    // actuators
    private TalonFX shooterMotor1;
    private CANSparkMax triggerMotor; // the shooter tower

    // sensors
    private DigitalInput ballDetector;
    private int direction = 0;
    private boolean now = false;
    private boolean lastBallDetector = false;

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
    private NetworkTableEntry normalShot = tab.add("normal shot preset", ShootK.normalShot).getEntry();
    private NetworkTableEntry againstHub = tab.add("against hub preset", ShootK.againstHub).getEntry();
    private NetworkTableEntry lowGoal = tab.add("low goal preset", ShootK.lowGoal).getEntry();
    private NetworkTableEntry farShot = tab.add("far shot preset", ShootK.farShot).getEntry();

    private IntakeSubsystem intake;
    
    /**
     * Makes a new ShooterSubsystem
     * the shooter controls the shooter motor(s?) and the "trigger motor"
     */
    public ShooterSubsystem(IntakeSubsystem intake) {
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
        }

        triggerMotor = new CANSparkMax(9, MotorType.kBrushless);
        triggerMotor.setIdleMode(IdleMode.kCoast);
        triggerMotor.setSmartCurrentLimit(30); // so as to not kill the baby neo
        ballDetector = new DigitalInput(9);

        this.intake = intake;
    }

    /**
     * sets the shooter to PID to the given velocity
     * @param RPM the rpm to set the shooter to
     */
    public void setShooter(double RPM) {
        if (shooterMotor1 != null) {
            // based off of tuning with pheonix tuner
            // WAIT HOLD UP I think this needs the secondary demand? of the PID loop?
            shooterMotor1.set(ControlMode.Velocity, RPM, DemandType.ArbitraryFeedForward, 0.4);
        }
    }

    public void stopShooter() {
        // coast the shooter down instead of PIDing it down which would maybe break it
        shooterMotor1.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * sets the shooter to the given preset value
     * @param preset the preset to set the shooter to
     */
    public void setShooterPreset(ShooterPresets preset) {
        if (preset == ShooterPresets.NORMAL_SHOT) {
            setShooter(normalShot.getDouble(ShootK.normalShot));
        } else if (preset == ShooterPresets.AGAINST_HUB) {
            setShooter(againstHub.getDouble(ShootK.againstHub));
        } else if (preset == ShooterPresets.LOW_GOAL) {
            setShooter(lowGoal.getDouble(ShootK.lowGoal));
        } else if (preset == ShooterPresets.FAR_SHOT) {
            setShooter(farShot.getDouble(ShootK.farShot));
        }
    }

    /**
     * returns if the shooter PID is at the last set setpoint or not
     * @return if the shooter is at its setpoint
     */
    public boolean atShooterSetpoint() {
        if (shooterMotor1 != null) {
            return Math.abs(shooterMotor1.getClosedLoopError()) < 100 && shooterMotor1.getErrorDerivative() < 10;
        }
        return true;
    }

    // sets the shooter tower ("trigger") motor with ball detection
    public void setTrigger(double power) {
        if (triggerMotor != null) {
            if (!ballDetector.get()) { // ball detector is inverse logic, so if we have ball
                if (power <= 0) { // let the ball move backwards
                    triggerMotor.set(power);
                    direction = -1;
                } else { // don't let the ball move forwards
                    triggerMotor.set(0);
                }
            } else {
                triggerMotor.set(power); // otherwise run as much as you want
            }
        }
    }

    // ignores ball detection, for when we want to actually shoot
    public void feed(double power) {
        if (triggerMotor != null) {
            triggerMotor.set(power);
            direction = 1;
        }
    }

    @Override
    public void periodic() {
        now = !ballDetector.get(); // make it to normal logic
        if (lastBallDetector && !now && direction == 1) {
            intake.decreaseCargoCount();
        }
        lastBallDetector = now;


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
                    shooterMotor1.config_kP(0, shootP.getDouble(ShootK.shootP));
                    shooterMotor1.config_kI(0, shootI.getDouble(ShootK.shootI));
                    shooterMotor1.config_kD(0, shootD.getDouble(ShootK.shootD));
                    shooterMotor1.config_kF(0, shootF.getDouble(ShootK.shootF));
                }
            }
        }
    }
}