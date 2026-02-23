
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
  private SparkMax feeder;
  private SparkMax bottomShooter;
  private SparkMax topShooter;
  /** Creates a new EndEffector. */
  public Shooter() {
    feeder =  new SparkMax(12, MotorType.kBrushless);
    bottomShooter =  new SparkMax(13, MotorType.kBrushless);
    SparkMaxConfig global_config = new SparkMaxConfig();
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    SparkMaxConfig bottomShooterConfig = new SparkMaxConfig();
    SparkMaxConfig topShooterConfig = new SparkMaxConfig();
    global_config
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    feederConfig
      .apply(global_config)      
      .inverted(true);
    bottomShooterConfig
      .apply(global_config)
      .inverted(false);

    feeder.configure(feederConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    bottomShooter.configure(bottomShooterConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    topShooter.configure(topShooterConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    /**
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    SmartDashboard.putBoolean("At Intake", atInSensor());
    
    SmartDashboard.putBoolean("At Outtake", atOutSensor());
    SmartDashboard.putBoolean("Acquired", acquired());
    */
  }
private void loadUp(){
  feeder.set(0.7);
}
private void stop(){
  feeder.set(0);
  bottomShooter.set(0);
  topShooter.set(0);
}
private void shoot(){
  bottomShooter.set(0.8);
  topShooter.set(0.8);
}

private void spitBack(){
  feeder.set(-0.25);
  bottomShooter.set(-0.25);
  topShooter.set(-.25);
}
public Command loadUpCommand(){
  return new RunCommand(this::loadUp, this).withName("loadUp");
}
public Command spitbackCommand(){
  return new RunCommand(this::spitBack, this).withName("spitBack");
}
public Command shootCommand(){
  return new RunCommand(this::shoot, this).withName("shoot");
}

/**public Command outtakeAndStopCommand(){
  return outtakeCommand()
  .until(() -> !this.acquired())
  .andThen(stopCommand())
  .withName("OuttakeAndStop");
}
*/

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