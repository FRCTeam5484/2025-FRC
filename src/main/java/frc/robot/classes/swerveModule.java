package frc.robot.classes;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class swerveModule {
    private final CANcoder rotationEncoder;
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;
    private final SparkClosedLoopController driveController;
    private final PIDController turnController;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private final double drivingFactor = Units.inchesToMeters(4) * Math.PI / 6.12; //WheelDiameterMeters * Math.PI / DrivingMotorReduction
    private final double driveWheelFreeSpeedRps = 6784 * (Units.inchesToMeters(4) * Math.PI) / 6.12;
    //private final double kDrivingMotorFreeSpeedRps = 6784;
    //private final double kWheelDiameterMeters = Units.inchesToMeters(4);
    //private final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    //private final double kDrivingMotorReduction = 6.12;
    //private final double kDriveWheelFreeSpeedRps = kDrivingMotorFreeSpeedRps / kDrivingMotorReduction;

    public swerveModule(int driveMotorPort, int turnMotorPort, int rotationEncoderPort, double angularOffset) {
        //Drive Motor
        driveMotor = new SparkMax(driveMotorPort, SparkMax.MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();        
        driveController = driveMotor.getClosedLoopController();
        driveConfig = new SparkMaxConfig();
        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .secondaryCurrentLimit(50);
        driveConfig.encoder
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor / 60.0);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0.0, 0.0)
            .velocityFF(1 / driveWheelFreeSpeedRps)
            .outputRange(-1, 1);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Turn Motor
        turnMotor = new SparkMax(turnMotorPort, SparkMax.MotorType.kBrushless);
        rotationEncoder = new CANcoder(rotationEncoderPort);
        turnController = new PIDController(0.01, 0, 0);
        turnController.enableContinuousInput(0, 360);
        turnController.setTolerance(1, 1);

        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.MagnetOffset = -angularOffset;
        canConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        rotationEncoder.getConfigurator().apply(canConfig);
        rotationEncoder.getPosition().setUpdateFrequency(100);
        rotationEncoder.getVelocity().setUpdateFrequency(100);

        turnConfig = new SparkMaxConfig();        
        turnConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .secondaryCurrentLimit(30);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        desiredState.angle = new Rotation2d(0);
        driveEncoder.setPosition(0);
    }
    public SwerveModuleState getState() { return new SwerveModuleState(driveEncoder.getVelocity(), getRotation2d()); }
    public double getAngle() { return rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360; }
    public double getRawAngle(){ return rotationEncoder.getAbsolutePosition().getValueAsDouble(); }
    public Rotation2d getRotation2d() { return Rotation2d.fromDegrees(getAngle()); }
    public void stopModule(){ driveMotor.set(0); turnMotor.set(0); }
    public SwerveModulePosition getPosition() { return new SwerveModulePosition(driveEncoder.getPosition(), getRotation2d()); }
    public void setDesiredState(SwerveModuleState state) { 
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;
        correctedDesiredState.optimize(getRotation2d());
        driveController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turnMotor.set(turnController.calculate(getAngle(), correctedDesiredState.angle.getDegrees()));
        desiredState = correctedDesiredState;
    }
}