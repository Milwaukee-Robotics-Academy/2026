import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter {
    private SparkMax m_motor_11;
    private SparkMax m_motor_12;
    private SparkMax m_motor_13;



    // this method controls the intake wheels
    public Shooter() {
        //initialize motor 9 as a SparkMax motor
        m_motor_12 = new SparkMax(12, MotorType.kBrushless);
        m_motor_13 = new SparkMax(13, MotorType.kBrushless);

        //set up configs for SparkMax motors
        SparkMaxConfig global_config = new SparkMaxConfig();
        SparkMaxConfig motor_12_config = new SparkMaxConfig();
        SparkMaxConfig motor_13_config = new SparkMaxConfig();

        //configure motor settings
        global_config
            //only use 40 if NOT 550 
            .smartCurrentLimit(40);
            //learn more about idle mode in the API docs
            .idleMode(IdleMode.kBrake);
            //learn more about persist mode in the API docs
            .persistMode(PersistMode.kPersist);

        motor_12_config
            .apply(global_config);
        motor_13_config
            .apply(global_config);

        m_motor_12.configure(motor_12_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_motor_13.configure(motor_12_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
}
