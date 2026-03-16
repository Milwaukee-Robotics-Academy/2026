// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
 
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       m_drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final FuelSubsystem m_fuelSubsystem = new FuelSubsystem();
  final PowerDistribution pdh = new PowerDistribution();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveStream = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.SCALE_TRANSLATION)
      .allianceRelativeControl(true)
      .scaleRotation(Constants.SCALE_ROTATION);

  SwerveInputStream defaultDriveStream = driveStream.copy()
      .aim(this.getHubPose())
      .aimHeadingOffset(true)
      .aimHeadingOffset(Rotation2d.k180deg) // Rotate the hub pose by 180 degrees to aim at the back of the hub
      .aimWhile(driverXbox.b());

    SwerveInputStream driveRotatingTowardsTravel = driveStream.copy()
      .headingWhile(true)
      .withControllerHeadingAxis(
        ()-> driverXbox.getLeftX() * -1, 
        ()-> driverXbox.getLeftY() * -1);

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
    NamedCommands.registerCommand("shoot", m_fuelSubsystem.runShooterCommand().withTimeout(3).withName("Auto Shoot"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", m_drivebase.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Register persistent Sendable items once (moved out of periodic)
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_drivebase);
    SmartDashboard.putData(m_fuelSubsystem);
    SmartDashboard.putData(pdh);
    SmartDashboard.putNumber("Clock/Match Time", 0.0);
    SmartDashboard.putBoolean("Clock/Shift 1 Active", false);
  }
  Pose2d getDrivingDirection() {
    return m_drivebase.getPose().plus(new Transform2d(driverXbox.getLeftX() * -10, driverXbox.getLeftY() * -10, m_drivebase.getPose().getRotation()));

  }
  
  Pose2d getHubPose() {
     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    // Set to red hub
    return new Pose2d(12, 4, new Rotation2d());
  } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    // Set to blue hub
    return new Pose2d(4.6, 4, new Rotation2d());
  } else {
    // do nothing
   return new Pose2d(); // Default value to avoid compile error; adjust as needed
  }
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

    Command defaultDriveStreamCommand = m_drivebase.driveFieldOriented(defaultDriveStream);
    Command driveRotatingTowardsTravelCommand = m_drivebase.driveFieldOriented(driveRotatingTowardsTravel);

    m_drivebase.setDefaultCommand(defaultDriveStreamCommand); // Overrides drive command above!

    driverXbox.x().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
    // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    driverXbox.start().onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
    driverXbox.back().whileTrue(m_drivebase.centerModulesCommand());

    // While the left bumper on operator controller is held, intake Fuel
    driverXbox.leftBumper().toggleOnTrue(m_fuelSubsystem.intakeCommand());
    operatorXbox.leftBumper().toggleOnTrue(m_fuelSubsystem.intakeCommand());

    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    driverXbox.rightBumper().toggleOnTrue(m_fuelSubsystem.runShooterCommand());
    operatorXbox.rightBumper().toggleOnTrue(m_fuelSubsystem.runShooterCommand());
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverXbox.a().whileTrue(m_fuelSubsystem.ejectCommand());
    operatorXbox.a().whileTrue(m_fuelSubsystem.ejectCommand());

    driverXbox.x().toggleOnTrue(driveRotatingTowardsTravelCommand);
    driverXbox.b().whileTrue(defaultDriveStreamCommand);

    m_fuelSubsystem.setDefaultCommand(m_fuelSubsystem.stopCommand());

 //   m_climberSubsystem.setDefaultCommand(m_climberSubsystem.run(() -> m_climberSubsystem.stop()));

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
    m_drivebase.setMotorBrake(brake);
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
    SmartDashboard.putData(m_drivebase);
    SmartDashboard.putData(m_fuelSubsystem);
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