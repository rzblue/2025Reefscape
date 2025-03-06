package frc.robot.subsystems;

import static frc.robot.Constants.CoralHeadConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class CoralHead extends SubsystemBase implements Logged {
  private final TalonFX motor = new TalonFX(motorID);
  private final DutyCycleOut dcRequest = new DutyCycleOut(0);

  private final DigitalInput rearSensor = new DigitalInput(rearSensorPort);
  private final DigitalInput midSensor = new DigitalInput(midSensorPort);
  private final DigitalInput frontSensor = new DigitalInput(frontSensorPort);

  public CoralHead() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(configuration);

    motor.getVelocity().setUpdateFrequency(50);
    motor.getPosition().setUpdateFrequency(50);
    motor.getDutyCycle().setUpdateFrequency(50);
    motor.getSupplyCurrent().setUpdateFrequency(25);
    motor.getStatorCurrent().setUpdateFrequency(25);
    motor.getDeviceTemp().setUpdateFrequency(4);
    motor.optimizeBusUtilization();
  }

  public void setOutput(double value) {
    dcRequest.Output = value;
    motor.setControl(dcRequest);
  }

  public void stop() {
    setOutput(0);
  }

  public Command outputCommand(double output) {
    return outputCommand(() -> output);
  }

  public Command outputCommand(DoubleSupplier output) {
    return run(() -> setOutput(output.getAsDouble())).finallyDo(this::stop);
  }

  public Command operatorIntake() {
    return outputCommand(intakeSpeed);
  }

  public Command slowIntake() {
    return outputCommand(slowSpeed);
  }

  public Command smartIntake() {
    return operatorIntake()
        .until(() -> coralAquired())
        .andThen(slowIntake())
        .until(() -> coralStowed());
  }

  public Command operatorScore() {
    return outputCommand(scoreSpeed);
  }

  public Command slowScore() {
    return outputCommand(.15);
  }

  public Command smartExtend() {
    return slowIntake().until(() -> coralExtended());
  }

  public Command smartScore() {
    return operatorScore().until(() -> coralClear());
  }

  public Command operatorRetract() {
    return outputCommand(retractSpeed);
  }

  public Command smartRetract() {
    return operatorRetract()
        .until(() -> coralAquired())
        .andThen(slowIntake())
        .until(() -> coralStowed());
  }

  @Log(key = "rear sensor")
  public boolean getRearSensor() {
    return rearSensor.get();
  }

  @Log(key = "mid sensor")
  public boolean getMidSensor() {
    return midSensor.get();
  }

  @Log(key = "front sensor")
  public boolean getFrontSensor() {
    return frontSensor.get();
  }

  public boolean coralClear() {
    return !getFrontSensor() && !getMidSensor() && !getRearSensor();
  }

  public boolean coralAquired() {
    return getMidSensor();
  }

  public boolean coralStowed() {
    return getMidSensor() && getFrontSensor();
  }

  public boolean coralExtended() {
    return getFrontSensor() && !getMidSensor() && !getRearSensor();
  }

  @Override
  public void periodic() {
    log("commanded output", dcRequest.Output);
    log("output", motor.getDutyCycle().getValueAsDouble());
    log("velocity", motor.getVelocity().getValueAsDouble());
    log("position", motor.getPosition().getValueAsDouble());
    log("stator current", motor.getStatorCurrent().getValueAsDouble());
    log("supply current", motor.getSupplyCurrent().getValueAsDouble());
    log("temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
