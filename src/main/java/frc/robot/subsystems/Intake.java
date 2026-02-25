package frc.robot.subsystems;

//import java.security.PrivateKey;
//import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

// import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase{
    
    
    private SparkFlex m_motor_9;  // intake motor
    private SparkMax m_motor_10;  // arm motor

    private SparkAbsoluteEncoder m_armEncoder;  //absolute encoder for arm position
    private SparkClosedLoopController m_armPID; //closed loop controller for arm position

    //arm position setpoints (play with these values to find best fit for the positions)
    private final double ARM_DOWN_POSITION = 0.16; 
    private final double ARM_MIDDLE_POSITION = 0.05;   
    private final double ARM_UP_POSITION = 0.87;   

    private static final double INTAKE_SPEED_FORWARD = 0.5; 
    private static final double INTAKE_SPEED_REVERSE = -0.5; 

    private static final double ARM_SPEED_MOVE_UP = -0.2;   //up = negative (arm is inverted)
    private static final double ARM_SPEED_MOVE_DOWN = 0.1;  //down = positive

    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================
    
    public Intake() {
        // initialize motor 9 as a SparkFlex motor and motor 10 as a SparkMax motor
        m_motor_9 = new SparkFlex(9, MotorType.kBrushless);
        m_motor_10 = new SparkMax(10, MotorType.kBrushless);

        // get absolute encoder and closed loop controller for arm motor
        m_armEncoder = m_motor_10.getAbsoluteEncoder();
        m_armPID = m_motor_10.getClosedLoopController();

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
             .positionConversionFactor(1.0)   // 1 rotation = 1.0 units
             .inverted(false);              // change to true if the encoder reads backwards

        motor_10_config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)     //use absolute encoder for closed loop control
            .pid(0.1, 0.0, 0.0)                            //tune these values for best performance
            .outputRange(-0.5, 0.5);                   //limit speed to 50%

        // apply configs to motors
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

    // ==================== SET ARM POSITION (ENCODER) ====================

    private void setArmPosition(double position) {
        m_armPID.setSetpoint(position, SparkMax.ControlType.kPosition);
    }

    public void moveArmDown() {
        setArmPosition(ARM_DOWN_POSITION);
    }
    public void moveArmMiddle() {
        setArmPosition(ARM_MIDDLE_POSITION);
    }
    public void moveArmUp() {
        setArmPosition(ARM_UP_POSITION);
    }   

    public double getArmPosition() {
        return m_armEncoder.getPosition();
    }

    // check if arm is at target position within a certain tolerance
    public boolean armAtTarget(double targetPosition) {
        double tolerance = 0.02; //within 0.02 rotations  
        return Math.abs(getArmPosition() - targetPosition) < tolerance;
    }

    // ==================== SET ARM POSITION (NO ENCODER) ====================

    public void armSpeedMoveUp() {
        m_motor_10.set(ARM_SPEED_MOVE_UP);
    }
    public void armSpeedMoveDown() {
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

    // ==================== ARM COMMANDS (ENCODER) ====================

    // public Command armDownCommand(){
    //     return new RunCommand(this::moveArmDown, this).withName("ArmDown");
    // }
    // public Command armMiddleCommand(){
    //     return new RunCommand(this::moveArmMiddle, this).withName("ArmMiddle");
    // }
    // public Command armUpCommand(){
    //     return new RunCommand(this::moveArmUp, this).withName("ArmUp");
    // }

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

        // Arm monitoring
        SmartDashboard.putNumber("Arm/Position", getArmPosition());
        SmartDashboard.putNumber("Arm/Current (A)", m_motor_10.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Temp (C)", m_motor_10.getMotorTemperature());
        
        // // Intake monitoring (NEO Vortex)
        SmartDashboard.putNumber("Intake/Current (A)", m_motor_9.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Temp (C)", m_motor_9.getMotorTemperature());
        SmartDashboard.putNumber("Intake/Voltage (V)", m_motor_9.getBusVoltage());
    }
}