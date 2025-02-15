package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class CoralHead extends SubsystemBase {
  private TalonFX motor = new TalonFX(51);
  private DutyCycleOut dcRequest = new DutyCycleOut(0);

  public CoralHead() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(configuration);
  }

  public void setOutput(double value) {
    dcRequest.Output = value;
    motor.setControl(dcRequest);
  }

  public void stop() {
    setOutput(0);
  }

  public Command outputCommand(DoubleSupplier output) {
    return run(() -> setOutput(output.getAsDouble())).finallyDo(this::stop);
  }
}
