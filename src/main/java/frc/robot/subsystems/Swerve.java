package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;
  public Canandgyro canandgyro;

  private StructPublisher<Pose2d> posePub =
      NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);

    canandgyro = new Canandgyro(0);
    canandgyro.resetFactoryDefaults(0.35);
    // CanandEventLoop.getInstance().setDevicePresenceWarnings(canandgyro, false);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    swerveOdometry =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      // TODO: handle gracefully
      throw new RuntimeException("Failed to load config");
    }
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getRobotRelativeChassisSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds), // driveRobotRelativeChassisSpeed(), // Method that will drive the
        // robot given ROBOT
        // RELATIVE ChassisSpeeds. Also optionally outputs
        // individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(Constants.AutoConstants.kPXController), // Translation PID constants
            new PIDConstants(Constants.AutoConstants.kPThetaController)),
        config,
        this::shouldFlipPath,
        this // Reference to this subsystem to set requirements
        );
  }

  public boolean shouldFlipPath() {
    // Boolean supplier that controls when the path will be mirrored for the red alliance
    // This will flip the path being followed to the red side of the field.
    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    swerveOdometry.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    swerveOdometry.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroYaw() {
    return canandgyro.getRotation2d();
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public Command teleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
    return run(
        () -> {
          /* Get Values, Deadband*/
          double translationVal = MathUtil.applyDeadband(x.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(y.getAsDouble(), Constants.stickDeadband);
          double rotationVal = MathUtil.applyDeadband(theta.getAsDouble(), Constants.stickDeadband);

          /* Drive */
          drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
              rotationVal * Constants.Swerve.maxAngularVelocity,
              true,
              true);
        });
  }

  @Override
  public void periodic() {

    swerveOdometry.update(getGyroYaw(), getModulePositions());
    posePub.set(swerveOdometry.getPoseMeters());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
  }
}

    // AutoBuilder.configureHolonomic(
    //   this::getPose,
    //   this::setPose,
    //   this::getRobotRelativeChassisSpeeds,
    //   this::driveRobotRelative,
    //   Constants.AutoConstants.pathFollowerConfig,
    //   this::shouldFlipPath,
    //   this);
