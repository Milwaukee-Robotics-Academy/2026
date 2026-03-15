// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {

  private SparkMax shooterFollower;
  private SparkMaxConfig shooterFollowerConfig;

  private SparkMax shooter;
  private SparkMaxConfig shooterConfig;
  private SparkClosedLoopController shooterController;

  private SparkMax indexer;
  private SparkMaxConfig indexerConfig;

  // Member variables for subsystem state management
  private double shooterTargetVelocity = 0.0;
  private RelativeEncoder shooterEncoder;

  private double kP = 0.0002;
  private double kV = 0.0021; // 5600 rpm is the free speed of a Neo at 12V, so this gives us volts per rpm.
  private double kA = 0.0; // You may need to tune this value based on how quickly you want the shooter to accelerate and decelerate.

  /**
   * Construct the CANFuelSubsystem.
   *
   * This sets up motor controllers for the intake/launcher and indexer, applies
   * default SmartDashboard tuning values, and configures current limits and
   * motor inversion where appropriate.
   */
  public FuelSubsystem() {

    SmartDashboard.putNumber("Shooter/kP", kP);
    SmartDashboard.putNumber("Shooter/kV", kV);
    SmartDashboard.putNumber("Shooter/kA", kA);

    shooter = new SparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterConfig = new SparkMaxConfig();
    shooterConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(0.5)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(80);
    shooterConfig.closedLoop
         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
         // Set PID values for position control
         .p(SmartDashboard.getNumber("Shooter/kP", kP))
         .outputRange(-1, 1);
    shooterConfig.encoder
        .uvwMeasurementPeriod(8)
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(8);
    shooterConfig.closedLoop.maxMotion
         // Set MAXMotion parameters for MAXMotion Velocity control
         .cruiseVelocity(5000)
         .maxAcceleration(10000);
    //     .allowedProfileError(1);
        
    // Nominal voltage is 12V, free speed of a Neo is 5600 rpm, so kV is 12V / 5600 rpm. Since feedforward.kV is in V/rpm,  
    // sort we take
    // the reciprocol.
    shooterConfig.closedLoop
            .feedForward.kV(SmartDashboard.getNumber("Shooter/kV", kV)); // 5600 rpm is the free speed of a Neo at 12V, so this gives us volts per rpm.
    shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterEncoder = shooter.getEncoder ();
    shooterController = shooter.getClosedLoopController();
    shooterEncoder = shooter.getEncoder();

    shooterFollower = new SparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterFollowerConfig = new SparkMaxConfig();
    shooterFollowerConfig.apply(shooterConfig)
        .follow(RIGHT_SHOOTER_MOTOR_ID, true);
    shooterFollower.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
    indexerConfig = new SparkMaxConfig();
    indexerConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(0.5)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(80);
    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Indexer/IntakingPercent", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Shooter/IntakingVelocity", SHOOTER_INTAKING_VELOCITY);
    SmartDashboard.putNumber("Indexer/ShootingPercent", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Shooter/ShootingVelocity", SHOOTER_SHOOTING_VELOCITY);
  }

  /**
   * Drive the flywheels to their set velocity. This will use MAXMotion
   * velocity control which will allow for a smooth acceleration and deceleration
   * to the mechanism's
   * setpoint.
   */
  private void setShooterVelocity(double velocity) {
    shooterController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
   // shooter.set(1);
    shooterTargetVelocity = velocity;
  }

  private void setIndexerPower(double power) {
    indexer.set(power);
  }

  private void intake() {
    indexer.set(Constants.FuelConstants.SHOOTER_INTAKING_VELOCITY);
    setShooterVelocity(SHOOTER_EJECT_VELOCITY);
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when
   * it reaches
   * the desired speed it starts the Feeder.
   */
  public Command runShooterCommand() {
    return this.startEnd(
        () -> this.setShooterVelocity(SHOOTER_SHOOTING_VELOCITY),
        () -> shooter.stopMotor()).until(isShooterAtSetpoint).andThen(
            this.startEnd(
                () -> {
                  this.setShooterVelocity(SHOOTER_SHOOTING_VELOCITY);
                  this.setIndexerPower(INDEXER_LAUNCHING_PERCENT);
                }, () -> {
                  shooter.stopMotor();
                  indexer.stopMotor();
                }))
        .withName("Shooting");
  }

  private void eject() {
    setShooterVelocity(SHOOTER_EJECT_VELOCITY);
    setIndexerPower(-INDEXER_INTAKING_PERCENT);
  }

  /**
   * Stop all rollers immediately.
   */
  public void stop() {
    shooter.stopMotor();
    indexer.stopMotor();
  }

  /**
   * Returns a command that runs the intake sequence while held.
   *
   * Values are read from SmartDashboard keys so they can be tuned at runtime.
   *
   * @return a Command that starts the intake and stops it when
   *         finished/interrupted
   */
  public Command intakeCommand() {
    return new RunCommand(
        this::intake,
        this).withName("Intake");
  }

  /**
   * Returns a command that runs intake+feeder in reverse to eject balls.
   *
   * @return a Command that runs the eject routine until interrupted
   */
  public Command ejectCommand() {
    return new RunCommand(this::eject, this).withName("Eject");
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Left-Velocity", shooterFollower.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Right-Velocity", shooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("Indexer/Velocity", indexer.getEncoder().getVelocity());
}

  public Command stopCommand() {
    return new RunCommand(this::stop, this).withName("Stop");
  }

  private boolean isShooterAt(double velocity) {
    return MathUtil.isNear(shooterEncoder.getVelocity(),
        velocity, 100);
  }

  /**
   * Trigger: Is the shooter spinning at the required velocity?
   */
  public final Trigger isShooterAtSetpoint = new Trigger(
      () -> isShooterAt(SHOOTER_SHOOTING_VELOCITY) || shooterEncoder.getVelocity() > SHOOTER_SHOOTING_VELOCITY);

}
