package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase
{
    private final QuestNav questNav = new QuestNav();

    @Override
    public void periodic() {
        questNav.commandPeriodic();
    }
    
}