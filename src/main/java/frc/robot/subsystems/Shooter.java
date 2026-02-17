
package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
  private SparkMax m_motor_12;
  private SparkMax m_motor_13;
  /** Creates a new EndEffector. */
  public Shooter() {
    m_motor_12 =  new SparkMax(12, MotorType.kBrushless);
    m_motor_13 =  new SparkMax(13, MotorType.kBrushless);
    SparkMaxConfig global_config = new SparkMaxConfig();
    SparkMaxConfig motor_12_config = new SparkMaxConfig();
    SparkMaxConfig motor_13_config = new SparkMaxConfig();
    global_config
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    motor_12_config
      .apply(global_config)      
      .inverted(true);
    motor_13_config
      .apply(global_config)
      .inverted(false);

    m_motor_12.configure(motor_12_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_motor_13.configure(motor_13_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    /**
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    SmartDashboard.putBoolean("At Intake", atInSensor());
    
    SmartDashboard.putBoolean("At Outtake", atOutSensor());
    SmartDashboard.putBoolean("Acquired", acquired());
    */
  }
private void loadUp(){
  m_motor_12.set(0.5);
  m_motor_13.set(0.5);
}
private void stop(){
  m_motor_12.set(0);
  m_motor_13.set(0);
}
private void shoot(){
  m_motor_12.set(0.5);
  m_motor_13.set(0.2);
}

private void spitBack(){
  m_motor_12.set(-0.5);
  m_motor_13.set(-0.5);
}
private void nudgeForward(){
  m_motor_12.set(0.1);
  m_motor_13.set(0.1);
}
public Command loadUpCommand(){
  return new RunCommand(this::loadUp, this).withName("loadUp");
}
public Command spitbackCommand(){
  return new RunCommand(this::spitBack, this).withName("spitBack");
}
public Command outtakeCommand(){
  return new RunCommand(this::shoot, this).withName("shoot");
}

/**public Command outtakeAndStopCommand(){
  return outtakeCommand()
  .until(() -> !this.acquired())
  .andThen(stopCommand())
  .withName("OuttakeAndStop");
}
*/
public Command nudgeForwardCommand(){
  return new RunCommand(this::nudgeForward, this).withName("nudge");
}

public Command stopCommand(){
 return new InstantCommand(this::stop, this).withName("Stopped");
}

/**public Command intakeWithSensorsCommand(){
  return this.intakeCommand()
  .until(()-> this.atInSensor())
  .andThen(this.nudgeForwardCommand())
  .until(() -> this.acquired())
  .andThen(this.stopCommand()).withName("IntakeWithSensors");
}
*/

/**
private boolean atInSensor(){
  return intakeSensor.getRange() <90;
}

private boolean atOutSensor(){
  return acquiredSensor.getRange() <90;
}


private boolean acquired(){
  if (!atInSensor() && atOutSensor()){
    return true;
  }
  return false;
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    SmartDashboard.putBoolean("At Intake", atInSensor());
    SmartDashboard.putBoolean("At Outtake", atOutSensor());
    SmartDashboard.putBoolean("Acquired", acquired());
  }
    */
}