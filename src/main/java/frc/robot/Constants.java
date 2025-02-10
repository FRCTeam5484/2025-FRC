package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class OperatorConstants {
      public static final int DriverOne = 0;
      public static final int DriverTwo = 1;
      public static final int StreamDeck = 2;
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
    }
    public static final class Elevator{
      public static final int LowerMotor = 13;
      public static final int UpperMotor = 14;
    }
    public static final class Algae {
      public static final int ArmMotor = 15;
      public static final int IntakeMotor = 16;
    }
    public static final class Coral {
      public static final int FeedMotor = 17;

      public static final int InSensor = 0;
      public static final int OutSensor = 1;
    }
}