package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final boolean isAlpha = RobotController.getSerialNumber().equals("03241508");

  public static final double stickDeadband = 0.04;

  public static final boolean aprilTagsEnabled = false;

  public static final AprilTagFieldLayout kOfficialField =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final class Swerve {
    public static final int pigeonID = 1;

    /** Drive motor rotations per rotation of azimuth */
    public static final double azimuthCouplingRatio = 50.0 / 14.0;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.75);
    public static final double wheelBase = Units.inchesToMeters(21.75);
    public static final double wheelCircumference = Units.inchesToMeters(3.798 * Math.PI);

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 150.0 / 7.0;

    /* Motor Inverts */

    /**
     * Direction the angle motor needs to go for the azimuth to move couterclockwise when viewed
     * from the top
     */
    public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;

    /** Direction the drive motor needs to go to drive the wheel "forwards" */
    public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

    /**
     * Angle Encoder Invert Encoder should read positive velocity when azimuth is rotated
     * counterclockwise when viewed from the top
     */
    public static final SensorDirectionValue cancoderInvert =
        SensorDirectionValue.CounterClockwise_Positive;

    /* Swerve Current Limiting */
    public static final int angleSupplyCurrentLimit = 25;
    public static final int angleSupplyCurrentLower = 40;
    public static final double angleSupplyCurrentLowerTime = 0.1;
    public static final boolean angleEnableSupplyCurrentLimit = true;
    public static final double angleStatorCurrentLimit = 80;
    public static final boolean angleEnableStatorCurrentLimit = true;

    public static final int driveSupplyCurrentLimit = 35;
    public static final int driveSupplyCurrentLower = 60;
    public static final double driveSupplyCurrentLowerTime = 0.1;
    public static final boolean driveEnableSupplyCurrentLimit = true;
    public static final double driveStatorCurrentLimit = 60;
    public static final boolean driveEnableStatorCurrentLimit = true;

    /* These values are used by the drive motor controller to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /*
     * Angle Motor PID Values
     * Feedback unit is rotations of azimuth
     * Output unit is voltage
     */
    public static final double angleKP = 100.0;
    public static final double angleKI = 0;
    public static final double angleKD = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot

    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isAlpha ? 79.189453 : 83.671875);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isAlpha ? -165.498047 : 124.541015625);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isAlpha ? 95.361328 : 28.564453125);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isAlpha ? -37.353516 : -71.71875);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AlgaeKickerConstants {
    public static final int motorID = 52;

    public static final double kickSpeed = 0.75;
  }

  public static final class CoralHeadConstants {
    public static final int motorID = 51;
    public static final int rearSensorPort = 10;
    public static final int midSensorPort = 11;
    public static final int frontSensorPort = 12;

    public static final double intakeSpeed = 0.15;
    public static final double slowSpeed = .075;
    public static final double scoreSpeed = 0.25;
    public static final double retractSpeed = -0.15;
  }

  public static final class ClimberConstants {
    public static final int servoPort = 0;
  }

  public static final class CameraConstants {
    public static final String reefCamName = "Arducam_OV9281_3";
    public static final Transform3d reefCamTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.375),
                Units.inchesToMeters(-10.25),
                Units.inchesToMeters(29.1875)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(12), Units.degreesToRadians(0)));

    public static final String coralCamName = "Arducam_OV9281_5";
    public static final Transform3d coralCamTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.625),
                Units.inchesToMeters(11.8125),
                Units.inchesToMeters(40.125)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-14),
                Units.degreesToRadians(210.5)));
  }

  public static final class AutoConstants {
    public static final double kPXController = 6;
    public static final double kPYController = 3;
    public static final double kPThetaController = 10;
  }
}
