package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import gg.questnav.questnav.QuestNav;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import gg.questnav.questnav.PoseFrame;
//import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;


import java.io.File;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.Matrix;

public class QuestNavSubsystem extends SubsystemBase
{
    QuestNav questNav = new QuestNav();
    // First, Declare our geometrical transform from the robot center to the Quest
    Transform3d ROBOT_TO_QUEST = new Transform3d(0, 0, 0, new Rotation3d());

    //SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    //                                                                            "swerve/maxSwerve"));
    Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
        );

    @Override
    public void periodic() {
        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                // You can put some sort of filtering here if you would like!

                // Add the measurement to our estimator
                //swerveDriveSubsystem.getSwerveDrive().addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        }
    }

    /*
    public void updatePoseEstimation(SwerveDrive swerveDrive)
    {
       PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
            for (PoseFrame frame : frames) {
                if (questNav.isTracking()) {
                Pose3d.addVisionMeasurement(
       frame.questPose3d(),      // Use the pose measurement
       frame.dataTimestamp());   // Use the timestamp of the measurement
                }
    }

    }
    */
    
}
