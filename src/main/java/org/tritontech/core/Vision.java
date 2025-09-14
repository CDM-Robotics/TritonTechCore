package org.tritontech.core;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

public class Vision extends SubsystemBase {

    static {
        VersionManager.initialize(); // Triggers VersionManager's static block
    }

    private final PhotonCamera camera;
    // private final PhotonCamera auxCamera;
    private final PhotonPoseEstimator photonEstimator;
    // private final PhotonPoseEstimator auxPhotonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private double lastEstTimestamp = 0;

    private Matrix<N3, N1> m_kSingleTagStdDevs;
    private Matrix<N3, N1> m_kMultiTagStdDevs;

    private double m_kCameraZOffset;
    private double m_kPitchOffset;

    public Vision(String cameraName, AprilTagFieldLayout kTagLayout, Transform3d kRobotToCam, double kCameraZOffset,
            double kPitchOffset) {
        m_kCameraZOffset = kCameraZOffset;
        m_kPitchOffset = kPitchOffset;

        camera = new PhotonCamera(cameraName);
        // auxCamera = new PhotonCamera(VisionConstants.kAuxPhoton);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);

        // auxPhotonEstimator =
        // new PhotonPoseEstimator(VisionConstants.kTagLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kAuxRobotToCam);

        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // auxPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // The default standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        m_kSingleTagStdDevs = VecBuilder.fill(2.5, 2.5, 5);
        m_kMultiTagStdDevs = VecBuilder.fill(0.06, 0.06, .12);
    }

    public void setSingleTagStdDevs(Matrix<N3, N1> single) {
        m_kSingleTagStdDevs = single;
    }

    public void setMultiTagStdDevs(Matrix<N3, N1> multi) {
        m_kMultiTagStdDevs = multi;
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        int i = 0;
        for (var change : camera.getAllUnreadResults()) {
            i++;
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }

        SmartDashboard.putNumber("Unreadresults", i);

        // Aux Camera Estimator
        /*
         * for (var change : auxCamera.getAllUnreadResults()) {
         * i++;
         * visionEst = auxPhotonEstimator.update(change);
         * updateEstimationStdDevs(visionEst, change.getTargets());
         * }
         */

        return visionEst;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = m_kSingleTagStdDevs;

        } else {
            SmartDashboard.putString("Std Devs", "STD DEV Found!!!");
            // Pose present. Start running Heuristic
            var estStdDevs = m_kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = m_kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = m_kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public boolean angFilter(int TagNum) {// tag num is the index number for the target Table
        return (new Rotation2d(Math.toRadians(180))
                .plus(getLatestResult().getTargets().get(TagNum).getBestCameraToTarget().getRotation().toRotation2d())
                .getDegrees() > 45
                || new Rotation2d(Math.toRadians(180)).plus(
                        getLatestResult().getTargets().get(TagNum).getBestCameraToTarget().getRotation().toRotation2d())
                        .getDegrees() < -45);
    }

    public double getTargetYaw() {
        return new Rotation2d(Math.toRadians(180))
                .plus(getLatestResult().getBestTarget().getBestCameraToTarget().getRotation().toRotation2d())
                .getDegrees();
    }

    public double getTargetingYaw() {
        double targetYaw = -9999.0;
        double targetDistance = 0;
        if (getLatestResult().hasTargets()) {

            for (PhotonTrackedTarget target : getLatestResult().getTargets()) {
                targetYaw = target.getYaw();
                // SmartDashboard.putNumber("Target Yaw (" + target.getFiducialId() + ")",
                // target.getYaw());
                targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
                        Units.inchesToMeters(m_kCameraZOffset), Units.inchesToMeters(8.75 + 6.5 / 2),
                        -m_kPitchOffset, target.pitch);
                // SmartDashboard.putNumber("Target Distance (" + target.getFiducialId() + ")",
                // targetDistance);
            }
        }

        return targetYaw;
    }
}
