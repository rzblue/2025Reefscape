package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePositionRequest = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset();

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID());
    angleEncoder.getConfigurator().apply(getEncoderConfig());

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID());
    mAngleMotor.getConfigurator().apply(getTurnMotorConfig());
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID());
    mDriveMotor.getConfigurator().apply(getDriveMotorConfig());
    mDriveMotor.getConfigurator().setPosition(0.0);
  }

  private TalonFXConfiguration getDriveMotorConfig() {
    var config = new TalonFXConfiguration();
    /* Motor Inverts and Neutral Mode */
    config.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
    config.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

    /* Gear Ratio Config */
    // swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

    /* Current Limiting */
    config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveSupplyCurrentLower;
    config.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveSupplyCurrentLowerTime;

    config.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.driveEnableStatorCurrentLimit;

    /* PID Config */
    config.Slot0.kP = Constants.Swerve.driveKP;
    config.Slot0.kI = Constants.Swerve.driveKI;
    config.Slot0.kD = Constants.Swerve.driveKD;

    config.Slot0.kS = Constants.Swerve.driveKS;
    // VS/m * m/r_w / (r_m/r_w) = VS/r_m
    config.Slot0.kA =
        Constants.Swerve.driveKA
            * Conversions.rotationsToMeters(1, Constants.Swerve.wheelCircumference)
            / Constants.Swerve.driveGearRatio;
    config.Slot0.kV =
        Constants.Swerve.driveKV
            * Conversions.rotationsToMeters(1, Constants.Swerve.wheelCircumference)
            / Constants.Swerve.driveGearRatio;

    /* Open and Closed Loop Ramping */
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    return config;
  }

  private TalonFXConfiguration getTurnMotorConfig() {
    var config = new TalonFXConfiguration();
    /* Motor Inverts and Neutral Mode */
    config.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
    config.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

    /* Gear Ratio and Wrapping Config */
    config.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
    config.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.angleSupplyCurrentLower;
    config.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.angleSupplyCurrentLowerTime;

    config.CurrentLimits.StatorCurrentLimit = Constants.Swerve.angleStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.angleEnableStatorCurrentLimit;

    /* PID Config */
    config.Slot0.kP = Constants.Swerve.angleKP;
    config.Slot0.kI = Constants.Swerve.angleKI;
    config.Slot0.kD = Constants.Swerve.angleKD;

    return config;
  }

  private CANcoderConfiguration getEncoderConfig() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
    return config;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState.optimize(getState().angle);

    // Cosine compensation
    var angleError = desiredState.angle.minus(getAngle());
    // Clamp scale factor to (0, 1) to prevent reversing
    // This shouldn't ever happen but just in case
    var velocityInDesiredDirection =
        MathUtil.clamp(angleError.getCos(), 0, 1) * desiredState.speedMetersPerSecond;

    setAngle(desiredState.angle);
    setSpeed(velocityInDesiredDirection, isOpenLoop);
  }

  private void setAngle(Rotation2d angle) {
    mAngleMotor.setControl(anglePositionRequest.withPosition(angle.getRotations()));
  }

  private void setSpeed(double speedMetersPerSecond, boolean isOpenLoop) {
    // Convert linear speed of wheel to motor speed
    var requestedVelocityRPS = wheelMeterToMotorRot(speedMetersPerSecond);
    // Calculate motor velocity required to hold wheel still at the current azimuth velocity
    // Don't compensate if requested velocity is 0 - just stop the motor
    double compensationVelocity = 0;
    if (speedMetersPerSecond != 0) {
      compensationVelocity =
          mAngleMotor.getVelocity().getValueAsDouble() * Constants.Swerve.azimuthCouplingRatio;
    }
    var outputVelocity = requestedVelocityRPS + compensationVelocity;

    if (isOpenLoop) {
      driveDutyCycleRequest.Output =
          outputVelocity / wheelMeterToMotorRot(Constants.Swerve.maxSpeed);
      mDriveMotor.setControl(driveDutyCycleRequest);
    } else {
      driveVelocityRequest.Velocity = outputVelocity;
      mDriveMotor.setControl(driveVelocityRequest);
    }
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        motorRotToWheelMeter(mDriveMotor.getVelocity().getValueAsDouble()), getAngle());
  }

  public SwerveModulePosition getPosition() {
    var driveRotations = mDriveMotor.getPosition().getValueAsDouble();
    // Calculate how many drive rotations were caused by azimuth coupling
    var azimuthCompensationDistance =
        getAngle().getRotations() * Constants.Swerve.azimuthCouplingRatio;
    // Subtract the "false" rotations from the recorded rotations to get the rotations that caused
    // wheel motion
    var trueDriveRotations = driveRotations - azimuthCompensationDistance;

    return new SwerveModulePosition(motorRotToWheelMeter(trueDriveRotations), getAngle());
  }

  public static double wheelMeterToMotorRot(double wheelMeters) {
    return Conversions.metersToRotations(wheelMeters, Constants.Swerve.wheelCircumference)
        * Constants.Swerve.driveGearRatio;
  }

  public static double motorRotToWheelMeter(double motorRot) {
    return Conversions.rotationsToMeters(
        motorRot / Constants.Swerve.driveGearRatio, Constants.Swerve.wheelCircumference);
  }
}
