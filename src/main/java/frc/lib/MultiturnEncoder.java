package frc.lib;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.MathUtil;

public class MultiturnEncoder {
  public final Canandmag encoderA;
  public final Canandmag encoderB;

  final double rangeCoefficient;

  double aOffset, bOffset;

  public MultiturnEncoder(int encoderAId, int encoderBId, int aTeeth, int bTeeth) {
    this.encoderA = new Canandmag(encoderAId);
    this.encoderB = new Canandmag(encoderBId);
    rangeCoefficient = (double) bTeeth / (double) (bTeeth - aTeeth);
    var settings =
        new CanandmagSettings()
            .setPositionFramePeriod(0.01)
            .setStatusFramePeriod(1)
            .setVelocityFramePeriod(0);
    encoderA.setSettings(settings);
    encoderB.setSettings(settings);
  }

  public double get() {
    var a = getA();
    var b = getB();
    var diff = a - b;
    return MathUtil.inputModulus(diff, 0, 1) * rangeCoefficient;
  }

  public void zero() {
    aOffset = getA();
    bOffset = getB();
  }

  public double getA() {
    return MathUtil.inputModulus(applyOffset(encoderA.getAbsPosition(), aOffset), 0, 1);
  }

  public double getB() {
    return MathUtil.inputModulus(
        applyOffset(MathUtil.inputModulus(-encoderB.getAbsPosition(), 0, 1), bOffset), 0, 1);
  }

  private static double applyOffset(double value, double offset) {
    return value - offset;
  }
}
