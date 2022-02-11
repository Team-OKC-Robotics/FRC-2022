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
import frc.robot.Constants.VisionK;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private Relay leds;
    private PIDController visionPID;

    //Shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("vision");
    private NetworkTableEntry toggleMode = tab.add("toggle camera", false).getEntry();
    

    public VisionSubsystem() {
        camera = new PhotonCamera("mmal_service_16.1");
        camera.setPipelineIndex(0);
        camera.setDriverMode(false);

        visionPID = new PIDController(VisionK.kP, VisionK.kI, VisionK.kD);
        visionPID.setSetpoint(0);

        leds = new Relay(2, Direction.kBoth);
    }

    /**
     * Returns the difference along the x axis between the center of the camera and the center of the target,
     * aka the yaw
     * @return the yaw of the best target
     */
    public double getXDifference() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return -result.getBestTarget().getYaw();
        }
        return 0;
    }

    /**
     * Resets the vision PID
     */
    public void resetVisionPID() {
        visionPID.reset();
    }

    /**
     * Gets the PID calculated output based on the yaw of the target
     * this would then be fed to the drivetrain
     * @return the output of the vision PID 
     */
    public double getOutput() {
        return visionPID.calculate(getXDifference());
    }

    /**
     * Gets if the vision PID is at its setpoint, ie we're aligned with the vision target
     * @return true if the camera is vision aligned 
     */
    public boolean atVisionSetpoint() {
        return visionPID.atSetpoint();
    }

    /**
     * Sets the LED ring that goes around the camera to light up the retroreflective vision target for vision tracking
     * @param on true to turn the LEDs on, false to turn off
     */
    public void setLeds(boolean on) {
        if (on) {
            leds.set(Value.kForward);
        } else {
            leds.set(Value.kOff);
        }
    }

    @Override
    public void periodic() {
        if (toggleMode.getBoolean(false)) {
            camera.setDriverMode(!camera.getDriverMode());
            toggleMode.setBoolean(false);
           // camera.setPipelineIndex(0);
        }
    }
}
