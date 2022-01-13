package frc.robot.subsystems;

public class ShooterSubsystem extends SubsystemBase {
    // actuators
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private PIDController shooterPID;

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
        shooterMotor1 = new CANSparkMax(MotorType.kBrushless, 20);
        shooterMotor2 = new CANSparkMax(MotorType.kBrushless, 21);

        shooterMotor2.follow(shooterMotor1);
        //TODO actually configure the motors and whatnot we DO NOT want to break anything

        shooterPID = new PIDController(ShootK.shootP, ShootK.shootI, ShootK.shootD, ShootK.shootF);
        // might be better to use the built-in PID loop or whatever (actually it's probably all in REVLib and not native)
        // for effeciency and optimization reasons (probably helps with CAN bus utilization)
        // but for now this will work
    }

    public void set(double RPM) {
        shooterMotor1.set(shooterPID.calculate(RPM, shooterMotor1.getVelocity()));
    }

    @Override
    public void periodic() {
        // update Shuffelboard values
        ticks.setDouble(shooterMotor1.get()); //TODO honestly idk what the motor methods need to be
        shooterRPM.setDouble(shooterMotor1.get()); //FIXME
        velocityError.setDouble(shooterMotor1.get()); //FIXME
        
        // Shuffleboard on-the-fly tuning
        if (writeMode.getBoolean(false)) {
            shooterPID.setP(shooterP.getDouble(ShootK.shootP));
            shooterPID.setI(shooterI.getDouble(ShootK.shootI));
            shooterPID.setD(shooterD.getDouble(ShootK.shootD));
            shooterPID.setF(shooterF.getDouble(ShootK.shootF));
        }
    }
}