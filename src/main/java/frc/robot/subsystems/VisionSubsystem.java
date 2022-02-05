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
    

    public VisionSubsystem() {
        camera = new PhotonCamera("mmal_service_16.1");
        camera.setPipelineIndex(0);
        camera.setDriverMode(false);

        visionPID = new PIDController(VisionK.kP, VisionK.kI, VisionK.kD);
        visionPID.setSetpoint(0);
    }

    public double getXDifference() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0;
    }

    public void resetVisionPID() {
        visionPID.reset();
    }

    public double getOutput() {
        return visionPID.calculate(getXDifference());
    }

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
