// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
//import frc.robot.subsystems.QuestNavSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() *-1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));
  private final PowerDistribution pdh = new PowerDistribution();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("shootCommand", m_Shooter.shootCommand().withTimeout(9));
    NamedCommands.registerCommand("loadUpCommand", m_Shooter.loadUpCommand().withTimeout(9));
    NamedCommands.registerCommand("intakeCommand", m_Intake.intakeCommand().withTimeout(5));
    NamedCommands.registerCommand("lowerArmCommand", m_Intake.goDownFunctionCommand().withTimeout(3));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    // if (RobotBase.isSimulation())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    // } else
    // {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
//     }

//     if (Robot.isSimulation())
//     {
//       Pose2d target = new Pose2d(new Translation2d(1, 4),
//                                  Rotation2d.fromDegrees(90));
//       //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
//       driveDirectAngleKeyboard.driveToPose(() -> target,
//                                            new ProfiledPIDController(5,
//                                                                      0,
//                                                                      0,
//                                                                      new Constraints(5, 2)),
//                                            new ProfiledPIDController(5,
//                                                                      0,
//                                                                      0,
//                                                                      new Constraints(Units.degreesToRadians(360),
//                                                                                      Units.degreesToRadians(180))
//                                            ));
//       driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
//       //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
//       //driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
//       //                                               () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      

// //      driverXbox.b().whileTrue(
// //          drivebase.driveToPose(
// //              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
// //                              );

//     }
//     if (DriverStation.isTest())
//     {
       drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
     m_Intake.setDefaultCommand(m_Intake.stopCommand());
     m_Shooter.setDefaultCommand(m_Shooter.stopCommand());

//       driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
//       //driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
//       driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
//       driverXbox.back().whileTrue(drivebase.centerModulesCommand());
//       driverXbox.leftBumper().onTrue(Commands.none());
//       driverXbox.rightBumper().onTrue(Commands.none());
//     } else
//     {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      //driverXbox.y().onTrue(drivebase.driveToDistanceCommandDefer(drivebase::getPose, 2, 14));
      driverXbox.y().whileTrue(drivebase.driveForward());
      operatorXbox.rightTrigger().whileTrue(m_Shooter.shootCommand());
      operatorXbox.rightBumper().whileTrue(m_Intake.intakeCommand());
      operatorXbox.leftTrigger().whileTrue(m_Shooter.loadUpCommand());
      operatorXbox.leftBumper().whileTrue(m_Intake.outtakeCommand());
      operatorXbox.x().whileTrue(m_Intake.goUpFunctionCommand());
      operatorXbox.y().whileTrue(m_Shooter.spitbackCommand());
      operatorXbox.a().whileTrue(m_Shooter.oppositeAgitateCommand());
      operatorXbox.b().whileTrue(m_Intake.goDownFunctionCommand());
      operatorXbox.povUp().whileTrue(m_Shooter.oppositeAgitateCommand()); 
      operatorXbox.povDown().onTrue(m_Intake.upSetCommand()); 
      operatorXbox.povLeft().whileTrue(m_Shooter.hyperShotCommand());
 //   }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  /**
 * Update SmartDashboard booleans (Shift 1..n) in one place.
 * Keeps the mapping and ranges consolidated.
 */
private void updateShiftStates(double matchTime) {
    // If matchTime < 0, treat all shifts as inactive (or choose different behavior above)
    boolean shift1Active = isBetween(matchTime, 30, 55);
    boolean shift2Active = isBetween(matchTime, 55, 80);
    boolean shift3Active = isBetween(matchTime, 80, 105);
    boolean shift4Active = isBetween(matchTime, 105, 120);
    boolean endgameShiftActive = isBetween(matchTime, 120, 150);
    boolean autoShiftActive = isBetween(matchTime, 0, 20);
    boolean transitionShiftActive = isBetween(matchTime, 20, 30);

    SmartDashboard.putBoolean("Clock/Hub Active?", isHubActive());
    SmartDashboard.putBoolean("Clock/Auto Active", autoShiftActive);
    SmartDashboard.putBoolean("Clock/Transition Active", transitionShiftActive);
    SmartDashboard.putBoolean("Clock/Shift 1 Active", shift1Active);
    SmartDashboard.putBoolean("Clock/Shift 2 Active", shift2Active);
    SmartDashboard.putBoolean("Clock/Shift 3 Active", shift3Active);
    SmartDashboard.putBoolean("Clock/Shift 4 Active", shift4Active);
    SmartDashboard.putBoolean("Clock/Endgame Active", endgameShiftActive);

}
public void periodic() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(drivebase);
    SmartDashboard.putData(m_Intake);
    SmartDashboard.putData(m_Shooter);
    SmartDashboard.putData(pdh);
    double matchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("Clock/Match Time", matchTime);
    updateShiftStates(matchTime);
}

private static boolean isBetween(double t, double startInclusive, double endExclusive) {
    return t >= startInclusive && t < endExclusive;
}
public boolean isHubActive() {
  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return true;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if (matchTime > 130) {
    // Transition shift, hub is active.
    return true;
  } else if (matchTime > 105) {
    // Shift 1
    return shift1Active;
  } else if (matchTime > 80) {
    // Shift 2
    return !shift1Active;
  } else if (matchTime > 55) {
    // Shift 3
    return shift1Active;
  } else if (matchTime > 30) {
    // Shift 4
    return !shift1Active;
  } else {
    // End game, hub always active.
    return true;
  }
}
}
