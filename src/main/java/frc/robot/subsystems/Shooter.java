
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private SparkMax shooter;
  private SparkMax disturbAgitator;
  private SparkMax spinAgitator;
  private SparkMax loader;
  /** Creates a new EndEffector. */
  public Shooter() {

    spinAgitator = new SparkMax(12, MotorType.kBrushless);
    shooter =  new SparkMax(14, MotorType.kBrushless);
    disturbAgitator =  new SparkMax(13, MotorType.kBrushless);
    loader = new SparkMax(15, MotorType.kBrushless);
    SparkMaxConfig global_config = new SparkMaxConfig();

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    global_config
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    shooterConfig
      .apply(global_config)
      .inverted(false);

   
    shooter.configure(shooterConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    /**
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    SmartDashboard.putBoolean("At Intake", atInSensor());
    
    SmartDashboard.putBoolean("At Outtake", atOutSensor());
    SmartDashboard.putBoolean("Acquired", acquired());
    */
  }
private void loadUp(){

  spinAgitator.set(0.1);
  disturbAgitator.set(-0.1);
  loader.set(0.25);
}
private void stop(){

  shooter.set(0);
  disturbAgitator.set(0);
  spinAgitator.set(0);
  loader.set(0);
}

private void oppositeAgitate(){
  spinAgitator.set(0.1);
  disturbAgitator.set(-0.1);
}
private void shoot(){
  shooter.set(-0.8);
}
private void hyperShot(){
  shooter.set(-1);
}

private void spitBack(){

  shooter.set(0.25);
  loader.set(0.1);
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