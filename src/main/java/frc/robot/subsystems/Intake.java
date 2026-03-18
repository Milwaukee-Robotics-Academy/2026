package frc.robot.subsystems;

import java.security.PrivateKey;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  private SparkMax m_rightArmMotor;
  private SparkMax m_leftArmMotor;
  private SparkMax m_intakeMotor;
  private SparkClosedLoopController m_leftArmController;
  private SparkClosedLoopController m_rightArmController;
  private RelativeEncoder m_rightArmEncoder;
  private RelativeEncoder m_leftArmEncoder;
  private RelativeEncoder m_intakeEncoder;
  private double leftArmTarget = 0;
  private double rightArmTarget = 0;

    

  public Intake(){
    m_rightArmMotor =  new SparkMax(9, MotorType.kBrushless);
    m_leftArmMotor =  new SparkMax(10, MotorType.kBrushless);
    m_intakeMotor =  new SparkMax(11, MotorType.kBrushless);
    m_rightArmEncoder = m_rightArmMotor.getEncoder();
    m_leftArmEncoder = m_leftArmMotor.getEncoder();
    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_leftArmController = m_leftArmMotor.getClosedLoopController();
    m_rightArmController = m_rightArmMotor.getClosedLoopController();

    //Setup Configuration of SparkMax Motors
    SparkMaxConfig global_config = new SparkMaxConfig();
    SparkMaxConfig rightArmMotorConfig = new SparkMaxConfig();
    SparkMaxConfig leftArmMotorConfig = new SparkMaxConfig();
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    global_config
      .smartCurrentLimit(0)
      .idleMode(IdleMode.kBrake);
    rightArmMotorConfig
      .apply(global_config);    
    rightArmMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(4)
        .i(0)
        .d(0)
        .positionWrappingEnabled(true);
    leftArmMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(4)
        .i(0)
        .d(0)
        .positionWrappingEnabled(true);
    leftArmMotorConfig
      .apply(global_config)
      .inverted(true);
    intakeMotorConfig
      .apply(global_config)
      .inverted(true);
    //Apply motor configuration to SparkMaxes
    m_rightArmMotor.configure(rightArmMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_leftArmMotor.configure(leftArmMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        // Zero arm encoder on initialization
    m_leftArmController.setReference(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_rightArmController.setReference(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Arm/Position", m_leftArmEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Target", leftArmTarget);
    SmartDashboard.putNumber("Arm/Current", m_leftArmMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm/Output", m_leftArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm/Voltage", m_leftArmMotor.getBusVoltage());
    SmartDashboard.putNumber("Arm/Position", m_rightArmEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Target", rightArmTarget);
    SmartDashboard.putNumber("Arm/Current", m_rightArmMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm/Output", m_rightArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm/Voltage", m_rightArmMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake/Current", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Output", m_intakeMotor.getAppliedOutput());

  }




  private void intake(){
    m_intakeMotor.set(0.85);
  }

  private void stopIntake(){
    m_intakeMotor.set(0);
    m_rightArmMotor.set(0);
    m_leftArmMotor.set(0);
  }

  private void outtake(){
    m_intakeMotor.set(-0.6);
  }

  private void raiseArm(){
    m_rightArmMotor.set(0.12);
    m_leftArmMotor.set(0.12);
  }

  private void stopArm(){
    m_rightArmMotor.set(0);
    m_leftArmMotor.set(0);
  }

  private void lowerArm(){
    m_rightArmMotor.set(-0.07);
    m_leftArmMotor.set(-0.07);
  }
  private void armDown(){
    m_rightArmEncoder.setPosition(2);
    m_leftArmEncoder.setPosition(2);
  }
  private void armUp(){
    m_rightArmEncoder.setPosition(1);
    m_leftArmEncoder.setPosition(1);
  }

  public Command intakeCommand(){
    return new RunCommand(this::intake, this).withName("Intake");
  }
  public Command outtakeCommand(){
    return new RunCommand(this::outtake, this).withName("Outtake");
  }

  public Command stopCommand(){
    return new InstantCommand(this::stopIntake, this).withName("StopIntake");
  }
  public Command goUpFunctionCommand(){
    return new RunCommand(this::raiseArm, this).withName("RaiseArm");
  }
  public Command goDownFunctionCommand(){
    return new RunCommand(this::lowerArm, this).withName("LowerArm");
  }
    
  public Command stop2Command(){
    return new InstantCommand(this::stopArm, this).withName("StopArm");
  }

  @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Arm Lifter Left", m_rightArmEncoder.getPosition());
      SmartDashboard.putNumber("Arm Lifter Right", m_leftArmEncoder.getPosition());
      SmartDashboard.putNumber("Intake", m_intakeEncoder.getPosition());
    
    }
}