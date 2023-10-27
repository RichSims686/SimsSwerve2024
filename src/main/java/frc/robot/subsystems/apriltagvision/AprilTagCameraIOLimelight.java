package frc.robot.subsystems.apriltagvision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.apriltagvision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.GeomUtil;

public class AprilTagCameraIOLimelight implements AprilTagCameraIO {

    private final String cameraName; 
    private final Transform3d robotToCamera;

    public AprilTagCameraIOLimelight(String cameraName, Transform3d robotToCamera) {
        // Important: need to configure robotToCamera pose using Limelight webUI
        // Important: need to configure AprilTag field map using Limelight webUI
        // https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#robot-localization-botpose-and-megatag
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        LimelightHelpers.setPipelineIndex(cameraName, 1);
    }

    public void updateInputs(AprilTagCameraIOInputs inputs) {
        // set default values
        inputs.robotToCamera = robotToCamera;
        inputs.isConnected = false;
        inputs.visionPose = Optional.empty();
        inputs.timestamp = Timer.getFPGATimestamp();
        inputs.cameraToTargets.clear();

        // get parsed results from JSON on NetworkTables.  
        // Use this JSON results to make sure all values are from the same snapshot
        LimelightHelpers.Results result = LimelightHelpers.getLatestResults(cameraName).targetingResults;

        // TODO: figure out how to determine if Limelight is disconnected
        inputs.isConnected = true;
        if (!inputs.isConnected) 
            return;

        if (!result.valid) {
            return;
        }

        // TODO: result.valid always returns true.  Use 'tv' from NetworkTables.  There is a small chance that the JSON and NT will be out of sync
        if (!LimelightHelpers.getTV(cameraName)) {
            return;
        }

        if (DriverStation.getAlliance().isPresent()) {
            Alliance alliance = DriverStation.getAlliance().get();
            if (alliance == Alliance.Blue) {
                inputs.visionPose = Optional.of(result.getBotPose3d_wpiBlue());
            } else if (alliance == Alliance.Red) {
                inputs.visionPose = Optional.of(result.getBotPose3d_wpiRed());
            }        
        }

        double latencySeconds = (result.latency_capture + result.latency_pipeline + result.latency_jsonParse) / 1000.0;
        inputs.timestamp = Timer.getFPGATimestamp() - latencySeconds;



        // populate targets array
        double limelightAmbiguity = 1.0;    // limelight doesn't give ambiguity, so set this as a large number
        for (LimelightTarget_Fiducial target : result.targets_Fiducials) {
            Pose3d pose = LimelightCoordinateTransform(target.getTargetPose_CameraSpace());
            inputs.cameraToTargets.add(
                new AprilTagTarget(
                    (int)target.fiducialID, 
                    GeomUtil.pose3dToTransform3d(pose),
                    limelightAmbiguity));
        }
    }

    /** Tranform from Limelight coordinates to Robot coordinates */
    /*  Limelight               : Robot
        x (camera right)        : -y (rightwards)
        y (camera up)           : +z (upwards)
        z (camera away)         : +x (forwards)
        rx (rotation about x)   : -pitch
        ry (rotation about y)   : +yaw
        rz (rotation about z)   : -roll
    */
    private Pose3d LimelightCoordinateTransform(Pose3d llPose) {
        Rotation3d llRot = llPose.getRotation();
        return new Pose3d(new Translation3d(+llPose.getZ(), -llPose.getX(), -llPose.getY()),
                          new Rotation3d(-llRot.getZ(), -llRot.getX(), +llRot.getY()));
    }
}
