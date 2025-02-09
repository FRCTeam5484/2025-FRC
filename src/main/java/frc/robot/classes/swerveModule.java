package frc.robot.classes;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class swerveModule {
    private final CANcoder rotationEncoder;
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public swerveModule(int driveMotorPort, int turnMotorPort, int rotationEncoderPort, double angularOffset) {
        //Drive Motor
        driveMotor = new SparkMax(driveMotorPort, SparkMax.MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();        
        driveController = driveMotor.getClosedLoopController();
        driveConfig = new SparkMaxConfig();
        driveConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        driveConfig.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Turn Motor
        turnMotor = new SparkMax(turnMotorPort, SparkMax.MotorType.kBrushless);
        rotationEncoder = new CANcoder(rotationEncoderPort);
        turnController = turnMotor.getClosedLoopController();

        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.MagnetOffset = -angularOffset;
        rotationEncoder.getConfigurator().apply(canConfig);
        rotationEncoder.getPosition().setUpdateFrequency(100);
        rotationEncoder.getVelocity().setUpdateFrequency(100);

        turnConfig = new SparkMaxConfig();        
        turnConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        turnConfig.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        desiredState.angle = new Rotation2d(0);
        driveEncoder.setPosition(0);
    }
    public SwerveModuleState getState() { return new SwerveModuleState(driveEncoder.getVelocity(), getRotation2d()); }
    public double getAngle() { return rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360; }
    public double getRawAngle(){ return rotationEncoder.getAbsolutePosition().getValueAsDouble(); }
    public Rotation2d getRotation2d() { return Rotation2d.fromDegrees(getAngle()); }
    public void stopModule(){ driveMotor.set(0); turnMotor.set(0); }
}