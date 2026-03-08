package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import static frc.robot.Constants.Vision.*;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken
 * from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2026RebuiltAndymark);
  /**
   * Ambiguity defined as a value between (0,1). Used in
   * {@link Vision#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the
   * april tag.
   */
  private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  private final PhotonCamera m_frontLeftCammera;
  private PhotonCamera m_rearRightCammera;
  private final PhotonPoseEstimator photonEstimatorLeft;
  private final PhotonPoseEstimator photonEstimatorRight;
  private Matrix<N3, N1> curStdDevs;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    m_frontLeftCammera = new PhotonCamera(kCameraNameFrontLeft);
    m_rearRightCammera = new PhotonCamera(kCameraNameRearRight);
    photonEstimatorLeft = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamFrontLeft);
    photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorRight = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamRearRight);
    photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    SmartDashboard.putBoolean("Use Vision?", true);
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive drive) {

    if (SmartDashboard.getBoolean("Use Vision?", false)) {
      // Correct pose estimate with vision measurements
      processVisionEstimation(this.getEstimatedGlobalPoseFromLeft(), drive, "VisionFrontLeft");
      processVisionEstimation(this.getEstimatedGlobalPoseFromRight(), drive, "VisionRearRight");
    }
  }
 private void processVisionEstimation(Optional<EstimatedRobotPose> poseEst, SwerveDrive drive, String visionLabel) {
    poseEst.ifPresent(est -> {
      // Change our trust in the measurement based on the tags we can see
      var estStdDevs = this.getEstimationStdDevs();

      // Checks if the values from vision are within field limits
      if ((est.estimatedPose.getTranslation().getY() > -0.05 && est.estimatedPose.getTranslation().getY() < 8.35) &&
        (est.estimatedPose.getTranslation().getX() > -0.05 && est.estimatedPose.getTranslation().getX() < 20.0)) {
        drive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      }
      SmartDashboard.putNumber(visionLabel + "/x", est.estimatedPose.getTranslation().getX());
      SmartDashboard.putNumber(visionLabel + "/y", est.estimatedPose.getTranslation().getY());
      SmartDashboard.putNumber(visionLabel + "/angle", est.estimatedPose.toPose2d().getRotation().getDegrees());
      SmartDashboard.putNumber(visionLabel + "/posetimestamp", est.timestampSeconds);
    });
  }
  

  /**
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   *
   * <p>
   * Also includes updates for the standard deviations, which can (optionally) be
   * retrieved with
   * {@link getEstimationStdDevs}
   *
   * @param camera The PhotonCamera to get the results from.
   * @param estimator The PhotonPoseEstimator to update the pose.
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera, PhotonPoseEstimator estimator) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      visionEst = estimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromLeft() {
    return getEstimatedGlobalPose(m_frontLeftCammera, photonEstimatorLeft);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromRight() {
    return getEstimatedGlobalPose(m_rearRightCammera, photonEstimatorRight);
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
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
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }


}