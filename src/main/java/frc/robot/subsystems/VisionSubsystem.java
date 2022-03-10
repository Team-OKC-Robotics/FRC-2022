package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
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
    private Relay leds;
    private PIDController visionPID;

    //Shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("vision");
    private NetworkTableEntry toggleMode = tab.add("toggle camera", false).getEntry();
    private NetworkTableEntry ledMode = tab.add("toggle leds", false).getEntry();

    
    /**
     * Makes a new VisionSubsystem
     * this subsystem houses the PhotonVision raspi + pi camera module
     * and is used to track the vision targets for automatic aiming
     */
    public VisionSubsystem() {
        camera = new PhotonCamera("mmal_service_16.1");
        camera.setPipelineIndex(1);
        camera.setDriverMode(false);

        visionPID = new PIDController(VisionK.kP, VisionK.kI, VisionK.kD);
        visionPID.setSetpoint(0);
        visionPID.setTolerance(0.5);

        // instatiate all the relays because for some reason this is the only way to one of them work
        leds = new Relay(0, Direction.kBoth);
        leds = new Relay(1, Direction.kBoth);
        leds = new Relay(3, Direction.kBoth);
        leds = new Relay(2, Direction.kBoth);

        leds.set(Value.kOff); // turn the leds off (yes I know it says kOn)
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
                return -result.getBestTarget().getYaw();
            }
        }
        return 0;
    }

    /**
     * resets the vision PID controller
     */
    public void resetVisionPID() {
        visionPID.reset();
    }

    /**
     * gets the output calculated by the PID controller based on the x difference
     * the drivetrain should make use of this number to align with the vision target
     * @return the output of the vision PID controller
     */
    public double getOutput() {
        return visionPID.calculate(getXDifference());
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
