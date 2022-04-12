package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbK;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonFX rightExtendMotor;
    private PIDController rightExtendPID;

    private WPI_TalonFX leftExtendMotor;
    private PIDController leftExtendPID;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("climber");
    private NetworkTableEntry writeMode = tab.add("Write Mode", false).getEntry();

    // sensor
    private NetworkTableEntry leftExtendPos = tab.add("Left Extend Position", 0).getEntry();

    private NetworkTableEntry rightExtendPos = tab.add("Right Extend Position", 0).getEntry();

    // PIDs (the presets can be the same though because it's the same hardware
    private NetworkTableEntry extendP = tab.add("extend kP", ClimbK.extendP).getEntry();
    private NetworkTableEntry extendI = tab.add("extend kI", ClimbK.extendI).getEntry();
    private NetworkTableEntry extendD = tab.add("extend kD", ClimbK.extendD).getEntry();
    
    // auto-climbing stuff
    private NetworkTableEntry leftExtendSetpoint = tab.add("left extend setpoint", 0).getEntry();
    private NetworkTableEntry rightExtendSetpoint = tab.add("right extend setpoint", 0).getEntry();

    private DataLog log;
    private DoubleLogEntry leftPosLog;
    private DoubleLogEntry leftOutputLog;

    private DoubleLogEntry rightPosLog;
    private DoubleLogEntry rightOutputLog;

    /**
     * Makes a new ClimberSubsystem
     * the climber consists (maybe) of some motors to move like an arm thing and also two winches
     * but also like idk what's going on
     */
    public ClimberSubsystem() {
        rightExtendMotor = new WPI_TalonFX(13);

        // set up the right side
        if (rightExtendMotor != null) {
            // set up the integrated sensor and make motor brake mode
            rightExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
            rightExtendMotor.setSelectedSensorPosition(0);
            rightExtendMotor.configNeutralDeadband(0);
            rightExtendMotor.setNeutralMode(NeutralMode.Brake);
            rightExtendMotor.configOpenloopRamp(0);

            // configure PID controller
            rightExtendPID = new PIDController(ClimbK.extendP, ClimbK.extendI, ClimbK.extendD);
            rightExtendPID.setTolerance(1000); // tolerance in ticks, might want to change to inches
        }

        // set up the left side
        leftExtendMotor = new WPI_TalonFX(15);
        
        if (leftExtendMotor != null) {
            // set up the integrated sensor and make motor brake mode
            leftExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
            leftExtendMotor.setSelectedSensorPosition(0);
            leftExtendMotor.configNeutralDeadband(0);
            leftExtendMotor.setNeutralMode(NeutralMode.Brake);
            leftExtendMotor.configOpenloopRamp(0.5);

            // configure PID controller
            leftExtendPID = new PIDController(ClimbK.extendP, ClimbK.extendI, ClimbK.extendD);
            leftExtendPID.setTolerance(1000); // tolerance in ticks, might want to change to inches
        }

        log = DataLogManager.getLog();
        leftPosLog = new DoubleLogEntry(log, "/climber/leftPos");
        leftOutputLog = new DoubleLogEntry(log, "/climber/leftOutput");
        rightPosLog = new DoubleLogEntry(log, "/climber/rightPos");
        rightOutputLog = new DoubleLogEntry(log, "/climber/rightOutput");
    }

    /**
     * sets the left winch? motor to the specified distance
     * @param ticks the distance to set the left winch motor to
     * physce it's actually not in inches it's just in ticks
     */
    public void setLeftExtend(double ticks) {
        if (leftExtendMotor != null) {
            leftExtendPID.setSetpoint(ticks);
            leftExtendMotor.set(TalonFXControlMode.PercentOutput, leftExtendPID.calculate(leftExtendMotor.getSelectedSensorPosition()));
        }
    }

    /**
     * sets the right winch? motor to the specified distance
     * @param ticks the distance to set the right winch motor to
     */
    public void setRightExtend(double ticks) {
        if (rightExtendMotor != null) {
            rightExtendPID.setSetpoint(ticks);
            rightExtendMotor.set(TalonFXControlMode.PercentOutput, rightExtendPID.calculate(rightExtendMotor.getSelectedSensorPosition()));
        }
    }


    // document all this stuff
    public boolean atLeftExtendSetpoint() {
        return leftExtendPID.atSetpoint();
    }

    public boolean atRightExtendSetpoint() {
        return rightExtendPID.atSetpoint();
    }

    //TODO document all the manual stuff    
    public void manualExtend(double power) {
        // sets both climbers so we don't have to change CAN IDs when we hot-swap the climbers
        if (leftExtendMotor != null) {
            leftExtendMotor.set(power);
            leftOutputLog.append(power);
        }

        if (rightExtendMotor != null) {
            rightExtendMotor.set(power); // just spooled it opposite direction so this doesn't need to be inverted
            rightOutputLog.append(power);
        }
    }

    public void extend(boolean leftSide) {
        if (leftSide) {
            leftExtendMotor.set(ControlMode.Position, leftExtendSetpoint.getDouble(ClimbK.extendLength));
        } else {
            rightExtendMotor.set(ControlMode.Position, rightExtendSetpoint.getDouble(ClimbK.extendLength));
        }
    }

    public void resetEncoders() {
        if (leftExtendMotor != null) {
            leftExtendMotor.setSelectedSensorPosition(0);
        } else if (rightExtendMotor != null) {
            rightExtendMotor.setSelectedSensorPosition(0);
        }
    }

    @Override
    public void periodic() {
        if (leftExtendMotor != null) {
            leftPosLog.append(leftExtendMotor.getSelectedSensorPosition());
        } else if (rightExtendMotor != null) {
            rightPosLog.append(rightExtendMotor.getSelectedSensorPosition());
        }


        // shuffleboard stuff
        if (!Constants.competition) {
            if (leftExtendMotor != null) {
                leftExtendPos.setDouble(leftExtendMotor.getSelectedSensorPosition());
            }
        
            if (rightExtendMotor != null) {
                rightExtendPos.setDouble(rightExtendMotor.getSelectedSensorPosition());
            }
        
        
            if (writeMode.getBoolean(false)) {
                if (leftExtendMotor != null) {
                    leftExtendPID.setP(extendP.getDouble(ClimbK.extendP));
                    leftExtendPID.setI(extendI.getDouble(ClimbK.extendI));
                    leftExtendPID.setD(extendD.getDouble(ClimbK.extendD));
                }
        
                if (rightExtendMotor != null) {
                    rightExtendPID.setP(extendP.getDouble(ClimbK.extendP));
                    rightExtendPID.setI(extendI.getDouble(ClimbK.extendI));
                    rightExtendPID.setD(extendD.getDouble(ClimbK.extendD));
                }
            }
        }
    }
}
