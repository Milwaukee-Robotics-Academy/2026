package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends SubsystemBase {
    
    private SparkMax m_motor_11;  // feeder motor

    private static final double FEEDER_SPEED_FORWARD = -0.7;
    private static final double FEEDER_SPEED_REVERSE = 0.5;

    // ==================== CONSTRUCTOR ====================

    public Feeder() {
        m_motor_11 = new SparkMax(11, MotorType.kBrushless);

        SparkMaxConfig motor_11_config = new SparkMaxConfig();
        motor_11_config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        m_motor_11.configure(motor_11_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ==================== FEEDER METHODS ====================

    private void forward() {
        m_motor_11.set(FEEDER_SPEED_FORWARD);
    }

    private void reverse() {
        m_motor_11.set(FEEDER_SPEED_REVERSE);
    }

    private void stop() {
        m_motor_11.set(0);
    }

    // ==================== COMMANDS ====================

    public Command forwardCommand() {
        return new RunCommand(this::forward, this).withName("ForwardFeeder");
    }

    public Command reverseCommand() {
        return new RunCommand(this::reverse, this).withName("ReverseFeeder");
    }

    public Command stopCommand() {
        return new RunCommand(this::stop, this).withName("StopFeeder");
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/Motor Output", m_motor_11.get());
        SmartDashboard.putBoolean("Feeder/Running", Math.abs(m_motor_11.get()) > 0.1);
    }
}