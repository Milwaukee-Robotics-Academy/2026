// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {

  private final SparkMax leftIntakeLauncher = new SparkMax(FuelConstants.LEFT_INTAKE_LAUNCHER_MOTOR_ID,
      MotorType.kBrushless);
  private final SparkMax rightIntakeLauncher = new SparkMax(FuelConstants.RIGHT_INTAKE_LAUNCHER_MOTOR_ID,
      MotorType.kBrushless);
  private final SmartMotorControllerConfig leftIntakeLauncherConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withFollowers(Pair.of(rightIntakeLauncher, true));
  private final SmartMotorController intakeLauncherController = new SparkWrapper(leftIntakeLauncher, DCMotor.getNEO(1),
      leftIntakeLauncherConfig);
  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(intakeLauncherController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-500), RPM.of(500))
      .withSpeedometerSimulation(RPM.of(750));
  private final FlyWheel intakeLauncherFlywheel = new FlyWheel(shooterConfig);

  private final SparkMax indexer = new SparkMax(FuelConstants.RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final SmartMotorControllerConfig indexerConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController indexerController = new SparkWrapper(indexer, DCMotor.getNEO(1),
      indexerConfig);


  /**
   * Construct the CANFuelSubsystem.
   *
   * This sets up motor controllers for the intake/launcher and indexer, applies
   * default SmartDashboard tuning values, and configures current limits and
   * motor inversion where appropriate.
   */
  public FuelSubsystem() {


    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    // SmartDashboard.putNumber("Spin-up feeder roller value",
    // SPIN_UP_FEEDER_VOLTAGE);
  }

  public AngularVelocity getVelocity() {
    return intakeLauncherFlywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return intakeLauncherFlywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double dutyCycle) {
    return intakeLauncherFlywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return intakeLauncherFlywheel.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return intakeLauncherFlywheel.set(dutyCycle);
  }

  public Command sysId() {
    return intakeLauncherFlywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
  }



  /**
   * Set the output for the indexer / feeder roller.
   *
   * @param power Motor output in the range supported by SparkMax (typically
   *              -1.0..1.0)
   */
  public void setFeederRoller(double power) {
    indexerController.setDutyCycle(power); // positive for shooting
  }

  /**
   * Stop all rollers immediately.
   */
  public void stop() {
    indexerController.setDutyCycle(0);
    intakeLauncherController.setVelocity(RPM.of(0));;
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
    return new edu.wpi.first.wpilibj2.command.StartEndCommand(
        () -> {
          double intakePercent = SmartDashboard.getNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
          double feederPercent = SmartDashboard.getNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
       //   setIntakeLauncherRoller(intakePercent);
          setFeederRoller(feederPercent);
        },
        this::stop,
        this).withName("Intake");
  }

  /**
   * Returns a command that runs intake+feeder in reverse to eject balls.
   *
   * @return a Command that runs the eject routine until interrupted
   */
  public Command ejectCommand() {
    return new edu.wpi.first.wpilibj2.command.StartEndCommand(
        () -> {
          double intakePercent = SmartDashboard.getNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
          double feederPercent = SmartDashboard.getNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
          // reverse the intake and feeder to eject
   // TODO       setIntakeLauncherRoller(-intakePercent);
          setFeederRoller(-feederPercent);
        },
        this::stop,
        this).withName("Eject");
  }

  /**
   * Returns a command to spin up the launcher wheels to a shooting setpoint.
   *
   * Launcher and feeder values are read from SmartDashboard.
   *
   * @return a Command that spins up the launcher until interrupted
   */
  public Command spinUpCommand() {
    return new edu.wpi.first.wpilibj2.command.StartEndCommand(
        () -> {
          double launcherPercent = SmartDashboard.getNumber("Launching launcher roller value",
              LAUNCHING_LAUNCHER_PERCENT);
          double feederPercent = SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    // TODO           setIntakeLauncherRoller(launcherPercent);
          setFeederRoller(feederPercent);
        },
        this::stop,
        this).withName("SpinUp");
  }

  /**
   * Returns a command that runs launcher+feeder for launching until interrupted.
   *
   * @return a Command that runs the full launch routine
   */
  // public Command launchCommand() {
  //   return new edu.wpi.first.wpilibj2.command.StartEndCommand(
  //       () -> {
  //         setIntakeLauncherRoller(
  //             SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
  //         setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
  //       },
  //       this::stop,
  //       this).withName("Launch");

  // }

  /**
   * Convenience sequence that spins up the launcher for a configured timeout,
   * then runs the full launch command.
   *
   * @return a SequentialCommandGroup performing spin-up then launch
   */
  public Command launchSequenceCommand() {
    // Spin up the launcher briefly, then run launcher+feeder to shoot until
    // interrupted
    return new edu.wpi.first.wpilibj2.command.SequentialCommandGroup(
        spinUpCommand().withTimeout(FuelConstants.SPIN_UP_SECONDS),
        launchSequenceCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
