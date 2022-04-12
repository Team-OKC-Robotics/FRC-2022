package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShootK;
import frc.robot.util.FrcUtil;

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
    private PIDController shooterPID;

    // sensors
    private DigitalInput ballDetector;
    private double power = 0;
    private double tempPower = 0;
    private LinearFilter rollingRpmAverage;
    private double averageRPM = 0;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("shooter");
    private NetworkTableEntry writeMode = tab.add("Write Mode", false).getEntry();
    
    // sensors
    private NetworkTableEntry ticks = tab.add("shooter ticks", 0).getEntry();
    private NetworkTableEntry shooterRPM = tab.add("shooter RPM", 0).getEntry();
    private NetworkTableEntry shooterOutput = tab.add("shooter output", 0).getEntry();
    private NetworkTableEntry setpoint = tab.add("setpoint", 0).getEntry();
    private NetworkTableEntry velocityError = tab.add("velocity error", 0).getEntry();
    private NetworkTableEntry hasBall = tab.add("has ball?", false).getEntry();

    // PID
    private NetworkTableEntry shootP = tab.add("Shooter kP", ShootK.shootP).getEntry();
    private NetworkTableEntry shootI = tab.add("Shooter kI", ShootK.shootI).getEntry();
    private NetworkTableEntry shootD = tab.add("Shooter kD", ShootK.shootD).getEntry();
    private NetworkTableEntry shooterGood = tab.add("shooter good", false).getEntry();

    // presets
    private NetworkTableEntry normalShot = tab.add("normal shot preset", ShootK.normalShot).getEntry();
    private NetworkTableEntry againstHub = tab.add("against hub preset", ShootK.againstHub).getEntry();
    private NetworkTableEntry lowGoal = tab.add("low goal preset", ShootK.lowGoal).getEntry();
    private NetworkTableEntry farShot = tab.add("far shot preset", ShootK.farShot).getEntry();

    private DataLog log;
    private DoubleLogEntry rpmLog;
    private DoubleLogEntry setpointLog;
    private DoubleLogEntry outputLog;
    private DoubleLogEntry calculatedLog;
    private DoubleLogEntry constantsLog;
    private BooleanLogEntry hasBallEntry;
    
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
            shooterMotor1.configOpenloopRamp(0);
            // shooterMotor1.config_kF(0, ShootK.shootF, 200);

            shooterMotor1.setSelectedSensorPosition(0);
        }

        triggerMotor = new CANSparkMax(9, MotorType.kBrushless);
        triggerMotor.setIdleMode(IdleMode.kCoast);
        triggerMotor.setSmartCurrentLimit(30); // so as to not kill the baby neo
        ballDetector = new DigitalInput(9);

        log = DataLogManager.getLog();
        rpmLog = new DoubleLogEntry(log, "/shooter/rpm");
        setpointLog = new DoubleLogEntry(log, "/shooter/setpoint");
        outputLog = new DoubleLogEntry(log, "/shooter/output");
        calculatedLog = new DoubleLogEntry(log, "/shooter/pid-calculate");
        constantsLog = new DoubleLogEntry(log, "/shooter/constants");
        hasBallEntry = new BooleanLogEntry(log, "/shooter/hasBall");
        constantsLog.append(ShootK.shootP);
        constantsLog.append(ShootK.shootI);
        constantsLog.append(ShootK.shootD);

        shooterPID = new PIDController(ShootK.shootP, ShootK.shootI, ShootK.shootD);
        shooterPID.setTolerance(100, 100); // tolerate a variance of 100 RPM and an acceleration of 10 RPM
        rollingRpmAverage = LinearFilter.movingAverage(7);
    }

    public void resetPower() {
        power = 0;
    }

    /**
     * sets the shooter to PID to the given velocity
     * @param RPM the rpm to set the shooter to
     */
    public void setShooter(double RPM) {
        if (shooterMotor1 != null) {
            // based off of tuning with pheonix tuner
            // shooterMotor1.set(ControlMode.Velocity, RPM, DemandType.ArbitraryFeedForward, 0.4);
            power += -shooterPID.calculate(RPM, shooterMotor1.getSelectedSensorVelocity());
            // if (atShooterSetpoint()) {
            //     // do nothing
            // } else {
            //     power += tempPower;
            // }
            shooterOutput.setDouble(FrcUtil.clamp(0.1, 1, power));
            setpoint.setDouble(RPM);
            setpointLog.append(RPM);
            shooterMotor1.set(ControlMode.PercentOutput, FrcUtil.clamp(0.1, 1, power));
            averageRPM = rollingRpmAverage.calculate(shooterMotor1.getSelectedSensorVelocity());
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
        return shooterPID.atSetpoint() && shooterPID.getSetpoint() != 0; // we're at the setpoint if we're at it and a setpoint is actually set
        // return shooterPID.getSetpoint() != 0 && Math.abs(averageRPM - shooterPID.getSetpoint()) < 100;
    }

    // sets the shooter tower ("trigger") motor with ball detection
    public void setTrigger(double power) {
        if (triggerMotor != null) {
            if (!ballDetector.get()) { // ball detector is inverse logic, so if we have ball
                if (power <= 0) { // let the ball move backwards
                    triggerMotor.set(power);
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
        }
    }

    @Override
    public void periodic() {
        rpmLog.append(shooterMotor1.getSelectedSensorVelocity());
        outputLog.append(FrcUtil.clamp(0.1, 1, power));
        shooterRPM.setDouble(shooterMotor1.getSelectedSensorVelocity());
        hasBallEntry.append(ballDetector.get());
        
        if (!Constants.competition) {
            hasBall.setBoolean(ballDetector.get());

            // update Shuffelboard values
            if (shooterMotor1 != null) {
                ticks.setDouble(shooterMotor1.getSelectedSensorPosition());
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
                    shooterPID.setP(shootP.getDouble(ShootK.shootP));
                    shooterPID.setI(shootI.getDouble(ShootK.shootI));
                    shooterPID.setD(shootD.getDouble(ShootK.shootD));
                }
            }
        }
    }
}