package frc.robot.subsystems;

//import java.security.PrivateKey;
//import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

// import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase{
    
    
    // ===private SparkMax m_motor_9; ===  // intake motor
    // ===private SparkMax m_motor_10; === // arm motor

    private SparkAbsoluteEncoder m_armEncoder;  //absolute encoder for arm position
    private SparkClosedLoopController m_armPID; //closed loop controller for arm position

    //arm position setpoints (play with these values to find best fit for the positions)
    private final double ARM_DOWN_POSITION = 0.2; 
    private final double ARM_MIDDLE_POSITION = 0.125;   
    private final double ARM_UP_POSITION = 0.0;   

    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================
    
    public Intake() {
        // //initialize motor 9 as a SparkMax motor
        // m_motor_9 = new SparkMax(9, MotorType.kBrushless);
        // m_motor_10 = new SparkMax(10, MotorType.kBrushless);

        // //set up configs for SparkMax motors
        // SparkMaxConfig global_config = new SparkMaxConfig();
        // SparkMaxConfig motor_9_config = new SparkMaxConfig();
        // SparkMaxConfig motor_10_config = new SparkMaxConfig();

        // //configure motor settings
        // global_config
        //     .smartCurrentLimit(40) // only use 40 if NOT 550 motor
        //     .idleMode(IdleMode.kBrake);

        // motor_9_config
        //     .apply(global_config);

        //  motor_10_config
        //     .apply(global_config);

        // motor_10_config.absoluteEncoder
        //      .positionConversionFactor(1.0)   // 1 rotation = 1.0 units
        //      .inverted(false);              // change to true if the encoder reads backwards

        // motor_10_config.closedLoop
        //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)     //use absolute encoder for closed loop control
        //     .pid(0.1, 0.0, 0.0)                            //tune these values for best performance
        //     .outputRange(-0.5, 0.5);                   //limit speed to 50%

        // m_motor_9.configure(motor_9_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        // m_motor_10.configure(motor_10_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    // ==================== SET INTAKE WHEEL SPEED ====================

    // private void forwardIntake() {
    //     m_motor_9.set(0.5);
    // }
    // private void reverseIntake() {
    //     m_motor_9.set(-0.5);
    // }
    // private void stopIntake() {
    //     m_motor_9.set(0);
    // }

    // ==================== SET ARM POSITION ====================

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

    // ==================== INTAKE WHEEL COMMANDS ====================

    // public Command forwardIntakeCommand(){
    //     return new RunCommand(this::forwardIntake, this).withName("ForwardIntake");
    // }
    // public Command reverseIntakeCommand(){
    //     return new RunCommand(this::reverseIntake, this).withName("ReverseIntake");
    // }
    // public Command stopIntakeCommand(){
    //     return new RunCommand(this::stopIntake, this).withName("StopIntake");
    // }

    // ==================== ARM COMMANDS ====================

    public Command armDownCommand(){
        return new RunCommand(this::moveArmDown, this).withName("ArmDown");
    }
    public Command armMiddleCommand(){
        return new RunCommand(this::moveArmMiddle, this).withName("ArmMiddle");
    }
    public Command armUpCommand(){
        return new RunCommand(this::moveArmUp, this).withName("ArmUp");
    }

    @Override 
    public void periodic() {
        // This method will be called once per scheduler run
        // You can use this to update SmartDashboard values or perform other periodic tasks
        SmartDashboard.putNumber("Arm Position", getArmPosition());
    }
}