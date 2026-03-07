
package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.opencv.features2d.AgastFeatureDetector;

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
  private SparkMax agitator;
  /** Creates a new EndEffector. */
  public Shooter() {
    feeder =  new SparkMax(15, MotorType.kBrushless);
    agitator = new SparkMax(11, MotorType.kBrushless);
    bottomShooter =  new SparkMax(13, MotorType.kBrushless);
    topShooter =  new SparkMax(14, MotorType.kBrushless);
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
    topShooterConfig
      .apply(global_config)
      .inverted(true);

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
  agitator.set(.1);
}
private void stop(){
  feeder.set(0);
  bottomShooter.set(0);
  topShooter.set(0);
  agitator.set(0);
}

private void oppositeAgitate(){
  agitator.set(-0.1);
}
private void shoot(){
  bottomShooter.set(0.58);
  topShooter.set(0.18);
}
private void hyperShot(){
  bottomShooter.set(1);
  topShooter.set(0.5);
}

private void spitBack(){
  feeder.set(-0.25);
  bottomShooter.set(-0.25);
  topShooter.set(-0.25);
}
public Command loadUpCommand(){
  return new RunCommand(this::loadUp, this).withName("loadUp");
}
public Command hyperShotCommand(){
  return new RunCommand(this::hyperShot, this).withName("hyperShot");
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
public Command oppositeAgitateCommand(){
 return new RunCommand(this::oppositeAgitate, this).withName("oppositeAgitate");
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