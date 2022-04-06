package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbK;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonFX rightExtendMotor;
    private PIDController rightExtendPID;
    private CANSparkMax rightTiltMotor;
    private RelativeEncoder rightTiltEncoder;
    private PIDController rightTiltPID;

    private WPI_TalonFX leftExtendMotor;
    private PIDController leftExtendPID;
    private CANSparkMax leftTiltMotor;
    private RelativeEncoder leftTiltEncoder;
    private PIDController leftTiltPID;

    // start out at zero and hold until otherwise told to
    private double leftSetpoint = 0;
    private double rightSetpoint = 0;
    private boolean leftStopped = true;
    private boolean rightStopped = true;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("climber");
    private NetworkTableEntry writeMode = tab.add("Write Mode", false).getEntry();

    // sensor
    private NetworkTableEntry leftTiltPos = tab.add("Left Tilt Position", 0).getEntry();
    private NetworkTableEntry leftExtendPos = tab.add("Left Extend Position", 0).getEntry();

    private NetworkTableEntry rightTiltPos = tab.add("Right Tilt Position", 0).getEntry();
    private NetworkTableEntry rightExtendPos = tab.add("Right Extend Position", 0).getEntry();

    // PIDs (the presets can be the same though because it's the same hardware
    private NetworkTableEntry extendP = tab.add("extend kP", ClimbK.extendP).getEntry();
    private NetworkTableEntry extendI = tab.add("extend kI", ClimbK.extendI).getEntry();
    private NetworkTableEntry extendD = tab.add("extend kD", ClimbK.extendD).getEntry();
    
    private NetworkTableEntry tiltP = tab.add("tilt kP", ClimbK.tiltP).getEntry();
    private NetworkTableEntry tiltI = tab.add("tilt kI", ClimbK.tiltI).getEntry();
    private NetworkTableEntry tiltD = tab.add("tilt kD", ClimbK.tiltD).getEntry();
        
    // auto-climbing stuff
    private NetworkTableEntry leftExtendSetpoint = tab.add("left extend setpoint", 0).getEntry();
    private NetworkTableEntry leftTiltSetpoint = tab.add("left tilt setpoint", 0).getEntry();
    private NetworkTableEntry rightExtendSetpoint = tab.add("right extend setpoint", 0).getEntry();
    private NetworkTableEntry rightTiltSetpoint = tab.add("right tilt setpoint", 0).getEntry();

    /**
     * Makes a new ClimberSubsystem
     * the climber consists (maybe) of some motors to move like an arm thing and also two winches
     * but also like idk what's going on
     */
    public ClimberSubsystem() {
        rightExtendMotor = new WPI_TalonFX(13);
        rightTiltMotor = new CANSparkMax(14, MotorType.kBrushless);

        // set up the right side
        if (rightExtendMotor != null) {
            // set up the integrated sensor and make motor brake mode
            rightExtendMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
            rightExtendMotor.setSelectedSensorPosition(0);
            rightExtendMotor.configNeutralDeadband(0);
            rightExtendMotor.setNeutralMode(NeutralMode.Brake);
            rightExtendMotor.configOpenloopRamp(0.5);

            // configure PID controller
            rightExtendPID = new PIDController(ClimbK.extendP, ClimbK.extendI, ClimbK.extendD);
            rightExtendPID.setTolerance(1000); // tolerance in ticks, might want to change to inches
        }

        if (rightTiltMotor != null) {
            // configure motor
            rightTiltMotor.setIdleMode(IdleMode.kBrake);

            // get and zero the encoder
            rightTiltEncoder = rightTiltMotor.getEncoder();
            rightTiltEncoder.setPosition(0);

            rightTiltMotor.setSoftLimit(SoftLimitDirection.kForward, 10);
            rightTiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        }
        
        // set up the left side
        leftExtendMotor = new WPI_TalonFX(15);
        leftTiltMotor = new CANSparkMax(16, MotorType.kBrushless);
        
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

        if (leftTiltMotor != null) {
            // configure motor
            leftTiltMotor.setIdleMode(IdleMode.kBrake);
            leftTiltMotor.setInverted(false);

            // get and zero encoder
            leftTiltEncoder = leftTiltMotor.getEncoder();
            leftTiltEncoder.setPosition(0);

            leftTiltMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
            leftTiltMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
            leftTiltMotor.setSoftLimit(SoftLimitDirection.kReverse, -16);
            leftTiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        }

        leftTiltPID = new PIDController(ClimbK.tiltP, ClimbK.tiltI, ClimbK.tiltD);
        rightTiltPID = new PIDController(ClimbK.tiltP, ClimbK.tiltI, ClimbK.tiltD);
    }

    /**
     * sets the left winch? motor to the specified distance
     * @param ticks the distance to set the left winch motor to
     * physce it's actually not in inches it's just in ticks
     */
    public void setLeftExtend(double ticks) {
        leftExtendPID.setSetpoint(ticks);
        leftExtendMotor.set(TalonFXControlMode.PercentOutput, leftExtendPID.calculate(leftExtendMotor.getSelectedSensorPosition()));
    }

    /**
     * Sets the left tilt motor to the specified angle, in degrees (probably)
     * @param angle the angle to set the left climber arm to 
     */
    public void setLeftTilt(double angle) {
        leftTiltMotor.set(leftTiltPID.calculate(angle, leftTiltEncoder.getPosition()));
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

    /**
     * Sets the right tilt motor to the specified angle, in degrees (probably)
     * @param angle the angle to set the right climber arm to 
     */
    public void setRightTilt(double angle) {
        rightTiltMotor.set(rightTiltPID.calculate(angle, rightTiltEncoder.getPosition()));
    }

    // document all this stuff
    public boolean atLeftExtendSetpoint() {
        return leftExtendPID.atSetpoint();
    }

    public boolean atLeftTiltSetpoint() {
        return leftTiltPID.atSetpoint();
    }

    public boolean atRightExtendSetpoint() {
        return rightExtendPID.atSetpoint();
    }

    public boolean atRightTiltSetpoint() {
        return rightTiltPID.atSetpoint();
    }

    //TODO document all the manual stuff    
    public void manualExtend(double power, boolean leftSide) {
        if (leftSide) {
            leftExtendMotor.set(power);
        } else {
            rightExtendMotor.set(power); // just spooled it opposite direction so this doesn't need to be inverted
        }
    }

    public void manualTilt(double power, boolean leftSide) {
        // if (Math.abs(power) > 0.05) { // only tilt if the change is significant
            if (leftSide) {
                leftTiltMotor.set(-power); // need to invert because opposite direction
            } else {
                rightTiltMotor.set(power);
            }   
        // }
    }

    // holds the climber in its position
    public void stopClimber(boolean leftSide) {
        if (leftSide) {
            leftSetpoint = leftTiltEncoder.getPosition();
            leftStopped = true;
            leftTiltMotor.set(0);
        } else {
            rightSetpoint = rightTiltEncoder.getPosition();
            rightStopped = true;
            leftTiltMotor.set(0);
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
        leftExtendMotor.setSelectedSensorPosition(0);
        rightExtendMotor.setSelectedSensorPosition(0);
        leftTiltEncoder.setPosition(0);
        rightTiltEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // if (leftTiltMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed()) {
        //     leftTiltEncoder.setPosition(0);
        //     leftSetpoint = 1;
        // }

        if (rightTiltMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed()) {
            rightTiltEncoder.setPosition(0);
            rightSetpoint = 1;
        }

        // shuffleboard stuff
        if (!Constants.competition) {            
            if (leftTiltEncoder != null) {
                leftTiltPos.setDouble(leftTiltEncoder.getPosition());
            }
        
            if (leftExtendMotor != null) {
                leftExtendPos.setDouble(leftExtendMotor.getSelectedSensorPosition());
            }
        
            if (rightTiltEncoder != null) {
                rightTiltPos.setDouble(rightTiltEncoder.getVelocity());
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
        
                leftTiltPID.setP(tiltP.getDouble(ClimbK.tiltP));
                leftTiltPID.setI(tiltI.getDouble(ClimbK.tiltI));
                leftTiltPID.setD(tiltD.getDouble(ClimbK.tiltD));
    
                if (rightExtendMotor != null) {
                    rightExtendPID.setP(extendP.getDouble(ClimbK.extendP));
                    rightExtendPID.setI(extendI.getDouble(ClimbK.extendI));
                    rightExtendPID.setD(extendD.getDouble(ClimbK.extendD));
                }
    
                rightTiltPID.setP(tiltP.getDouble(ClimbK.tiltP));
                rightTiltPID.setI(tiltI.getDouble(ClimbK.tiltI));
                rightTiltPID.setD(tiltD.getDouble(ClimbK.tiltD));
            }
        }
    }
}
