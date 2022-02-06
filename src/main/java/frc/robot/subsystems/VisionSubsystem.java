package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionK;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PIDController visionPID;

    //Shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("vision");
    private NetworkTableEntry toggleMode = tab.add("toggle camera", false).getEntry();
    
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
        return visionPID.atSetpoint();
    }

    @Override
    public void periodic() {
        if (toggleMode.getBoolean(false)) {
            camera.setDriverMode(!camera.getDriverMode());
            toggleMode.setBoolean(false);
        }
    }
}
