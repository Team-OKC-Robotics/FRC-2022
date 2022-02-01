package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionK;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PIDController visionPID;

    public VisionSubsystem() {
        camera = new PhotonCamera("photonvision");
        camera.setPipelineIndex(0);
        camera.setDriverMode(false);

        visionPID = new PIDController(VisionK.kP, VisionK.kI, VisionK.kD);
        visionPID.setSetpoint(0);
    }

    public double getXDifference() {
        if (camera != null) {
            PhotonPipelineResult result = camera.getLatestResult();
    
            if (result.hasTargets()) {
                return result.getBestTarget().getYaw();
            }
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
}
