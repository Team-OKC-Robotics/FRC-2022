package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionK;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonCamera driverCamera;
    private Relay leds;
    private PIDController visionPID;
    private SlewRateLimiter slewRateLimiter;

    //Shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("vision");
    private NetworkTableEntry toggleMode = tab.add("toggle camera", false).getEntry();
    private NetworkTableEntry ledMode = tab.add("toggle leds", false).getEntry();

    private DataLog log;
    private DoubleLogEntry xDifferenceLog;
    private DoubleLogEntry visionConstantsLog;

    
    /**
     * Makes a new VisionSubsystem
     * this subsystem houses the PhotonVision raspi + pi camera module
     * and is used to track the vision targets for automatic aiming
     */
    public VisionSubsystem() {
        camera = new PhotonCamera("mmal_service_16.1");
        camera.setDriverMode(false);
        camera.setPipelineIndex(0);

        driverCamera = new PhotonCamera("HD_USB_CAMERA");
        driverCamera.setDriverMode(true);

        visionPID = new PIDController(VisionK.kP, VisionK.kI, VisionK.kD);
        visionPID.setSetpoint(0);
        visionPID.setTolerance(0.5, 0.1);

        // instatiate all the relays because for some reason this is the only way to one of them work
        leds = new Relay(0, Direction.kBoth);
        leds = new Relay(1, Direction.kBoth);
        leds = new Relay(3, Direction.kBoth);
        leds = new Relay(2, Direction.kBoth);

        leds.set(Value.kOff); // turn the leds off (yes I know it says kOn)
        
        log = DataLogManager.getLog();
        xDifferenceLog = new DoubleLogEntry(log, "/vision/xdifference");
        visionConstantsLog = new DoubleLogEntry(log, "/vision/constants");
        visionConstantsLog.append(VisionK.kP);
        visionConstantsLog.append(VisionK.kI);
        visionConstantsLog.append(VisionK.kD);

        slewRateLimiter = new SlewRateLimiter(1.5);
    }

    /**
     * returns the distance along the x-axis between the center of the camera and the center of the vision target
     * may need to be offset by a value later depending on where the camera is positioned
     * @return the error along the x-axis of the vision target and crosshair
     */
    public double getXDifference() {
        if (camera != null) {
            PhotonPipelineResult result = camera.getLatestResult();
    
            if (result.hasTargets()) {
                return -result.getBestTarget().getYaw() - 0.1;
            }
        }
        return 0;
    }

    /**
     * resets the vision PID controller
     */
    public void resetVisionPID() {
        visionPID.reset();
        // slewRateLimiter.reset(0);
    }

    /**
     * gets the output calculated by the PID controller based on the x difference
     * the drivetrain should make use of this number to align with the vision target
     * @return the output of the vision PID controller
     */
    public double getOutput() {
        return visionPID.calculate(getXDifference());
        // return slewRateLimiter.calculate(visionPID.calculate(getXDifference()));
    }

    /**
     * returns if the vision PID controller is at its setpoint or not
     * essentially, if we are aligned with the vision target or not
     * @return true if the robot is aligned with the vision target
     */
    public boolean atVisionSetpoint() {
        if (getOutput() == 0) {
            return false;
        }
        //return false;
        return visionPID.atSetpoint();
    }

    /**
     * Sets the LED ring that goes around the camera to light up the retroreflective vision target for vision tracking
     * @param on true to turn the LEDs on, false to turn off
     */
    public void setLeds(boolean on) {
        if (on) {
            leds.set(Value.kForward);
            ledMode.setBoolean(true);
        } else {
            leds.set(Value.kOff);
            ledMode.setBoolean(false);
        }
    }

    /**
     * Gets the pitch of the vision target relative to the center of the camera
     * @return the pitch of the vision target
     */
    public double getPitch() {
        PhotonPipelineResult result = camera.getLatestResult();
    
        if (result.hasTargets()) {
            return result.getBestTarget().getPitch();
        }
        return 0;
    }

    /**
     * Gets the distance to the vision target, for use by the Shooter to warm up to the right RPM
     * so we can arc into the goal
     * @return the distance to the vision target
     */
    public double getDistance() {
        return (VisionK.heightOfGoal - VisionK.heightOfCamera) / Math.tan(VisionK.cameraAngle + getPitch());
    }

    @Override
    public void periodic() {
        xDifferenceLog.append(getXDifference());

        if (!Constants.competition) {
            // leds.set(ledMode.getBoolean(false) ? Value.kForward : Value.kOff); // I think that's how you use ternary
            // idk I use Python man
    
            if (toggleMode.getBoolean(false)) {
                camera.setDriverMode(!camera.getDriverMode());
                toggleMode.setBoolean(false);
               // camera.setPipelineIndex(0);
            }
        }
    }
}