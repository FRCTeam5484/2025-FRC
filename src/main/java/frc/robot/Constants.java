package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);

    public static final class OperatorConstants {
      public static final int DriverOne = 0;
      public static final int ButtonBoxControllerOne = 1;
      public static final int ButtonBoxControllerTwo = 2;
      public static final int ButtonBoxControllerThree = 3;
    }
    public static final class DriveConstants {
      public static final double kMaxSpeedMetersPerSecond = 5.88264;
      public static final double kMaxAngularSpeed = 4 * Math.PI;
    }
    public static final class SwerveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        public static final double kWheelBase = Units.inchesToMeters(26.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        //The following values are only for reference.  You must change them in the Src -> Deploy -> Swerve -> *.json files
        public static final double kFrontLeftOffset = 0.93652;
        public static final double kFrontRightOffset = 0.42968;
        public static final double kRearLeftOffset = 0.04614;
        public static final double kRearRightOffset = 0.61659;
      
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kRearLeftDrivingCanId = 7;
        public static final int kRearRightDrivingCanId = 10;
      
        public static final int kFrontLeftTurningCanId = 2;
        public static final int kFrontRightTurningCanId = 5;
        public static final int kRearLeftTurningCanId = 8;
        public static final int kRearRightTurningCanId = 11;
      
        public static final int kFrontLeftCANcoder = 3;
        public static final int kFrontRightCANcoder = 6;
        public static final int kRearLeftCANcoder = 9;
        public static final int kRearRightCANcoder = 12;

        public static final int Pigeon2 = 1;
    }
    public static final class Elevator{
      public static final int LowerMotor = 13;
      public static final int UpperMotor = 14;
      public static final int LowerLimitFront = 3;
      public static final int LowerLimitBack = 5;
      public static final int UpperLimitFront = 6;
      public static final int UpperLimitBack = 4;
      //public static final int StringPot = 8,9;

      public static final double bottomPosition = -5;
      public static final double L1 = 500;
      public static final double L2 = 920;
      public static final double L3 = 3000;
      public static final double L4 = 6150;
      public static final double topPosition = 6300;
    }
    public static final class Algae {
      public static final int RemoverMotor = 18;

      public static final double RemoverArmBack = 0;
      public static final double RemoverArmUp = 5700;
      public static final double RemoverArmDown = 13700;
    }
    public static final class Coral {
      public static final int FeedMotor = 17;

      public static final int InSensor = 0;
      public static final int OutSensor = 1;
    }
    public static final class Climb {
      public static final int hookMotor = 19;
      public static final int winchMotor = 20;
    }
    public static final class LimeLightOffsets{
      public static final class Left{
        public static final double HorizontalOffset = 5.08;
        public static final double DistanceOffset = 13.63;
      }
      public static final class Right{
        public static final double HorizontalOffset = -20.78;
        public static final double DistanceOffset = 13.01;
      }
    }
    public static final class PosePositions{
      public static final class Blue{
        public static final Pose2d BackLeft = new Pose2d(new Translation2d(5.792, 3.756), new Rotation2d(0));
        public static final Pose2d BackRight = new Pose2d(new Translation2d(5.803, 3.990), new Rotation2d(0));
        public static final Pose2d LeftBackLeft = new Pose2d(new Translation2d(5.512, 5.117), new Rotation2d(45));
        public static final Pose2d LeftBackRight = new Pose2d(new Translation2d(5.504, 4.957), new Rotation2d(45));
        public static final Pose2d LeftFrontLeft = new Pose2d(new Translation2d(4.128, 5.336), new Rotation2d(135));
        public static final Pose2d LeftFrontRight = new Pose2d(new Translation2d(3.839, 5.157), new Rotation2d(135));
        public static final Pose2d FrontLeft = new Pose2d(new Translation2d(3.171, 4.399), new Rotation2d(180));
        public static final Pose2d FrontRight = new Pose2d(new Translation2d(3.151, 4.020), new Rotation2d(180));
        public static final Pose2d RightFrontLeft = new Pose2d(new Translation2d(3.530, 3.053), new Rotation2d(225));
        public static final Pose2d RightFrontRight = new Pose2d(new Translation2d(3.809, 2.913), new Rotation2d(225));
        public static final Pose2d RightBackLeft = new Pose2d(new Translation2d(4.856, 2.704), new Rotation2d(270));
        public static final Pose2d RightBackRight = new Pose2d(new Translation2d(5.115, 2.883), new Rotation2d(270));
      }
      public static final class Red{
        public static final Pose2d BackLeft = new Pose2d(new Translation2d(11.75, 3.675), new Rotation2d(0));
        public static final Pose2d BackRight = new Pose2d(new Translation2d(11.75, 3.987), new Rotation2d(0));
        public static final Pose2d LeftBackLeft = new Pose2d(new Translation2d(12.697, 2.710), new Rotation2d(45));
        public static final Pose2d LeftBackRight = new Pose2d(new Translation2d(12.366, 2.891), new Rotation2d(45));
        public static final Pose2d LeftFrontLeft = new Pose2d(new Translation2d(13.764, 2.876), new Rotation2d(135));
        public static final Pose2d LeftFrontRight = new Pose2d(new Translation2d(14.064, 3.041), new Rotation2d(135));
        public static final Pose2d FrontLeft = new Pose2d(new Translation2d(14.395, 4.048), new Rotation2d(180));
        public static final Pose2d FrontRight = new Pose2d(new Translation2d(14.395, 4.393), new Rotation2d(180));
        public static final Pose2d RightFrontLeft = new Pose2d(new Translation2d(13.749, 5.205), new Rotation2d(225));
        public static final Pose2d RightFrontRight = new Pose2d(new Translation2d(13.388, 5.370), new Rotation2d(225));
        public static final Pose2d RightBackLeft = new Pose2d(new Translation2d(12.111, 5.039), new Rotation2d(270));
        public static final Pose2d RightBackRight = new Pose2d(new Translation2d(12.351, 5.144), new Rotation2d(270));
      }
    }
    // Motor IDs
      
      // Front Left Drive = 1
      // Front Left Turn = 2
      // Front Right Drive = 4
      // Front Right Turn = 5
      // Rear Left Drive = 7
      // Rear Left Turn = 8
      // Rear Right Drive = 10
      // Rear Right Turn = 11

      // Elevator Lower Motor = 13
      // Elevator Upper Motor = 14
      
      // Coral Feed Motor = 17
      // Algae Remover Motor = 18      

    // CAN Sensors

      // Front Left CANcoder = 3
      // Front Right CANcoder = 6
      // Rear Left CANcoder = 9
      // Rear Right CANcoder = 12
      // Pigeon = 0

    // Digital Inputs

      // Coral In Sensor = 0
      // Coral Out Sensor = 1
      // Elevator Lower Limit Front = 3
      // Elevator Upper Limit Front = 4
      // Elevator Lower Limit Back = 5
      // Elevator Upper Limit Back = 6
      // Elevator String Pot Port 1 = 8
      // Elevator String Pot Port 2 = 9
}