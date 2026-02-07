package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;


public class QuestNavSubsystem extends SubsystemBase
{
    private final QuestNav questNav = new QuestNav();

    @Override
    public void periodic() {
        questNav.commandPeriodic();
    }

    //Edit this once Quest is mounted to robot. Measurements from robot center to Quest
    Transform3d ROBOT_TO_QUEST = new Transform3d(/*add X, Y, Z, yaw, pitch, roll offsets here*/);
    
    //Get latest pose data frames from Quest
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
        Pose3d questPose = poseFrames[poseFrames.length - 1].questPose();

        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
    }
}