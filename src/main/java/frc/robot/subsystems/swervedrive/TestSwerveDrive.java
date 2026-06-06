package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;

public class TestSwerveDrive extends SwerveDrive {

  private SwerveDrivePoseEstimator visionPoseEstimator;
  private SwerveDrivePoseEstimator questPoseEstimator;

  public TestSwerveDrive(SwerveDriveConfiguration config, SwerveControllerConfiguration controllerConfig,
      double maxSpeedMPS, Pose2d startingPose) {
    super(config, controllerConfig, maxSpeedMPS, startingPose);
    visionPoseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getYaw(),
        getModulePositions(),
        startingPose);
    questPoseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getYaw(),
        getModulePositions(),
        startingPose);
  }
  

  /**
   * Add a vision measurement to the vision pose estimator.
   * 
   * @param visionRobotPose    The pose measured by the vision system
   * @param timestampSeconds   The timestamp of the vision measurement
   * @param visionMeasurement  Standard deviation of the vision measurement
   */
  public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds,
      Matrix<N3, N1> visionMeasurement) {
    visionPoseEstimator.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurement);
  }

  /**
   * Add a quest pose measurement to the quest pose estimator.
   * 
   * @param questRobotPose    The pose measured by the quest system
   * @param timestampSeconds  The timestamp of the quest measurement
   * @param questMeasurement  Standard deviation of the quest measurement
   */
  public void addQuestMeasurement(Pose2d questRobotPose, double timestampSeconds,
      Matrix<N3, N1> questMeasurement) {
    questPoseEstimator.addVisionMeasurement(questRobotPose, timestampSeconds, questMeasurement);
  }

  /**
   * Get the estimated pose from the vision pose estimator.
   * 
   * @return The estimated robot pose from vision measurements
   */
  public Pose2d getVisionEstimatedPose() {
    return visionPoseEstimator.getEstimatedPosition();
  }

  /**
   * Get the estimated pose from the quest pose estimator.
   * 
   * @return The estimated robot pose from quest measurements
   */
  public Pose2d getQuestEstimatedPose() {
    return questPoseEstimator.getEstimatedPosition();
  }

}
