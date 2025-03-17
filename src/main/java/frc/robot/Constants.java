package frc.robot;

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
      public static final int DriverTwo = 1;
      public static final int ButtonBoxControllerOne = 2;
      public static final int ButtonBoxControllerTwo = 3;
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

      public static final double bottomPosition = -50;
      public static final double L1 = 500;
      public static final double L2 = 1100;
      public static final double L3 = 3100;
      public static final double L4 = 6250;
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
      public static final double HorizontalOffset = 15.46;
      public static final double DistanceOffset = 7.36;
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