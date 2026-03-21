package frc.robot.subsystems;

//import java.security.PrivateKey;
//import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
//import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase{
    
    
    private SparkFlex m_motor_9;  // intake motor
    private SparkMax m_motor_10;  // arm motor

    private SparkAbsoluteEncoder m_armEncoder;

    // TODO: Set DIO port
    private static final int ARM_DOWN_LIMIT_SWITCH_PORT = 0; 
    private static final int ARM_UP_LIMIT_SWITCH_PORT = 1;  
    
    private static final double ARM_DOWN_POSITION = 0.822;   
    private static final double ARM_UP_POSITION = 0.359;    

    private static final double ARM_POSITION_TOLERANCE = 0.0;  // Adjust based on testing

    private DigitalInput m_downLimitSwitch;
    private DigitalInput m_upLimitSwitch;

    private static final double INTAKE_SPEED_FORWARD = 0.4; 
    private static final double INTAKE_SPEED_REVERSE = -0.7; 

    private static final double ARM_SPEED_MOVE_UP = -0.3;   //up = negative (arm is inverted)
    private static final double ARM_SPEED_MOVE_DOWN = 0.1;  //down = positive

    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================
    
    public Intake() {
        // initialize motor 9 as a SparkFlex motor and motor 10 as a SparkMax motor
        m_motor_9 = new SparkFlex(9, MotorType.kBrushless);
        m_motor_10 = new SparkMax(10, MotorType.kBrushless);

        m_downLimitSwitch = new DigitalInput(ARM_DOWN_LIMIT_SWITCH_PORT);
        m_upLimitSwitch = new DigitalInput(ARM_UP_LIMIT_SWITCH_PORT);

        m_armEncoder = m_motor_10.getAbsoluteEncoder();

        // set up configs for SparkFlex motor 9 (intake)
        SparkFlexConfig global_config_flex = new SparkFlexConfig();
        SparkFlexConfig motor_9_config = new SparkFlexConfig();

        // set up configs for SparkMax motor 10 (arm)
        SparkMaxConfig global_config_max = new SparkMaxConfig();
        SparkMaxConfig motor_10_config = new SparkMaxConfig();

        // configure flex motor settings (intake)
        global_config_flex
            .smartCurrentLimit(40) // 60-80 for Vortex
            .idleMode(IdleMode.kBrake);

        motor_9_config
            .apply(global_config_flex);

        // configure max motor settings (arm)
        global_config_max
            .smartCurrentLimit(40) // 40-60 for NEO
            .idleMode(IdleMode.kBrake);

        motor_10_config
            .apply(global_config_max);

        motor_10_config.absoluteEncoder
            .positionConversionFactor(1.0)
            .inverted(true); // Invert encoder to match motor direction

        m_motor_9.configure(motor_9_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_motor_10.configure(motor_10_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    // ==================== SET INTAKE WHEEL SPEED ====================

    private void forwardIntake() {
        m_motor_9.set(INTAKE_SPEED_FORWARD);
    }
    private void reverseIntake() {
        m_motor_9.set(INTAKE_SPEED_REVERSE);
    }
    private void stopIntake() {
        m_motor_9.set(0);
    }

    // ==================== CHECKS LIMIT POSITION ====================

    public boolean isAtDownLimitSwitch() {
        return !m_downLimitSwitch.get();   // return true when pressed
    }
    public boolean isAtUpLimitSwitch() {
        return !m_upLimitSwitch.get();     // return true when pressed
    }

    // ==================== CHECKS ENCODER POSITION ====================
    
    public double getArmPosition() {
        return m_armEncoder.getPosition();
    }
    public boolean isAtDownEncoderLimit() {
        double position = getArmPosition();
        return position >= (ARM_DOWN_POSITION + ARM_POSITION_TOLERANCE);
    }

    public boolean isAtUpEncoderLimit() {
        double position = getArmPosition();
        return position <= (ARM_UP_POSITION - ARM_POSITION_TOLERANCE);
    }

    // ==================== CHECKS LIMITS ====================
    
    public boolean isAtUpLimit() {
        return isAtUpLimitSwitch() || isAtUpEncoderLimit();  
        //return isAtUpLimitSwitch();
    }

    public boolean isAtDownLimit() {
        return isAtDownLimitSwitch() || isAtDownEncoderLimit();  
        //return isAtDownLimitSwitch();
    }
      // ==================== MOVES ARM ====================

  public void armSpeedMoveUp() {
        if (isAtUpLimit()) {
            m_motor_10.set(0);
            return;
        }
        m_motor_10.set(ARM_SPEED_MOVE_UP);
    }
    
    public void armSpeedMoveDown() {
        if (isAtDownLimit()) {
            m_motor_10.set(0);
            return;
        }     
        m_motor_10.set(ARM_SPEED_MOVE_DOWN);
    }
    
    public void stopArm() {
        m_motor_10.set(0);
    }

        // ==================== INTAKE WHEEL COMMANDS ====================

    public Command forwardIntakeCommand(){
        return new RunCommand(this::forwardIntake, this).withName("ForwardIntake");
    }
    public Command reverseIntakeCommand(){
        return new RunCommand(this::reverseIntake, this).withName("ReverseIntake");
    }
    public Command stopIntakeCommand(){
        return new InstantCommand(this::stopIntake, this).withName("StopIntake");
    }


    // ==================== ARM COMMANDS (NO ENCODER) ====================

    public Command armSpeedUpCommand(){
        return new RunCommand(this::armSpeedMoveUp, this).withName("ArmSpeedUp");
    }
    public Command armSpeedDownCommand(){
        return new RunCommand(this::armSpeedMoveDown, this).withName("ArmSpeedDown");
    }
    public Command stopArmCommand(){
        return new InstantCommand(this::stopArm, this).withName("StopArm");
    }

    // ==================== ARM & INTAKE COMMANDS (NO ENCODER) ====================

    public Command stopAllCommand() {
    return new RunCommand(() -> {
        stopIntake();
        stopArm();
    }, this).withName("StopAll");
}

    // ==================== PRINT STATEMENTS ====================

    @Override 
    public void periodic() {
        // This method will be called once per scheduler run
        // You can use this to update SmartDashboard values or perform other periodic tasks
        SmartDashboard.putNumber("Encoder", m_armEncoder.getPosition());

        SmartDashboard.putBoolean("Down Switch", isAtDownLimitSwitch());
        SmartDashboard.putBoolean("Up Switch", isAtUpLimitSwitch());

        SmartDashboard.putBoolean("Down Encoder", isAtDownEncoderLimit());
        SmartDashboard.putBoolean("Up Encoder", isAtUpEncoderLimit());

        SmartDashboard.putBoolean("At Down Limit", isAtDownLimit());
        SmartDashboard.putBoolean("At Up Limit", isAtUpLimit());

    }
}