package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends SubsystemBase implements Logged {
  private final TalonFX leader = new TalonFX(41);
  private final TalonFX follower = new TalonFX(42);

  private final Follower followReq = new Follower(leader.getDeviceID(), false);
  private final MotionMagicVoltage posRequest = new MotionMagicVoltage(0);

  public Elevator() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.CurrentLimits.withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLowerLimit(40)
        .withSupplyCurrentLimitEnable(true);

    follower.getConfigurator().apply(config);

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(52.4)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
        .withReverseSoftLimitEnable(true);

    config.Slot0.withKP(3);
    config.MotionMagic.withMotionMagicAcceleration(400).withMotionMagicCruiseVelocity(100);

    leader.getConfigurator().apply(config);
    follower.setControl(followReq);
    leader.setPosition(0);
    leader.getStatorCurrent().setUpdateFrequency(50);
    leader.getSupplyCurrent().setUpdateFrequency(50);
    leader.getVelocity().setUpdateFrequency(50);
    leader.getDutyCycle().setUpdateFrequency(100);
    leader.optimizeBusUtilization();
  }

  public void setGoal(double position) {
    leader.setControl(posRequest.withPosition(position));
  }

  public Command positionCommand(double position) {
    return runOnce(() -> setGoal(position));
  }

  public Command relativePositionCommand(double relativePos) {
    return runOnce(() -> setGoal(posRequest.Position + relativePos));
  }

  @Log(key = "Position")
  public double getPosition() {
    return leader.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    log("Elevator/Velocity", leader.getVelocity().getValueAsDouble());
    log("Elevator/StatorCurrent", leader.getStatorCurrent().getValueAsDouble());
    log("Elevator/SupplyCurrent", leader.getSupplyCurrent().getValueAsDouble());
  }
}
