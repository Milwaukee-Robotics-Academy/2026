package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Matrix;
import swervelib.SwerveDrive;



public class QuestNavSubsystem extends SubsystemBase
{
    QuestNav questNav = new QuestNav();

    @Override
    public void periodic() {
        questNav.commandPeriodic();
    }

    Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put your x, y, z, yaw, pitch, and roll offsets here!*/ );

    public void poseOdometry(SwerveDrive swerve) {
        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        
        for (PoseFrame frame : frames){
            if (frame.isTracking()){
                Pose3d questPose = frame.questPose3d();
                double timestamp = frame.dataTimestamp();

                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                swerve.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);

            }
        }
    }
    
}
