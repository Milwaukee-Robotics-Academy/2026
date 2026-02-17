package frc.robot.subsystems;

import java.security.PrivateKey;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
  private SparkMax m_motor_9;
  private SparkMax m_motor_10;
  private SparkMax m_motor_11;
  private RelativeEncoder m_encoder_9;
  private RelativeEncoder m_encoder_10;
  private RelativeEncoder m_encoder_11;

    

  public Intake(){
    m_motor_9 =  new SparkMax(11, MotorType.kBrushless);
    m_motor_10 =  new SparkMax(12, MotorType.kBrushless);
    m_encoder_9 = m_motor_9.getEncoder();
    m_encoder_10 = m_motor_10.getEncoder();

    //Setup Configuration of SparkMax Motors
    SparkMaxConfig global_config = new SparkMaxConfig();
    SparkMaxConfig motor_9_config = new SparkMaxConfig();
    SparkMaxConfig motor_10_config = new SparkMaxConfig();
    global_config
      .smartCurrentLimit(0)
      .idleMode(IdleMode.kBrake);
    motor_9_config
      .apply(global_config);    
    motor_10_config
      .apply(global_config);
    //Apply motor configuration to SparkMaxes
    m_motor_9.configure(motor_9_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_motor_10.configure(motor_10_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  }



  private void intake(){
    m_motor_11.set(0.5);
  }

  private void stopIntake(){
    m_motor_11.set(0);
  }

  private void outtake(){
    m_motor_11.set(-0.5);
  }

  private void raiseArm(){
    m_motor_9.set(0.25);
    m_motor_10.set(0.25);
  }

  private void stopArm(){
    m_motor_9.set(0);
    m_motor_10.set(0);
  }

  private void lowerArm(){
    m_motor_9.set(-0.25);
    m_motor_10.set(-0.25);
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
      SmartDashboard.putNumber("Arm Lifter Left", m_encoder_9.getPosition());
      SmartDashboard.putNumber("Arm Lifter Right", m_encoder_10.getPosition());
      SmartDashboard.putNumber("Intake", m_encoder_11.getPosition());
    
    }
}