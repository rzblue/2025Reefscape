package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {

  Swerve drivetrain;

  public Command runPath(String pathName, boolean resetPose) {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Could not load path '" + pathName + "'", e.getStackTrace());
      return Commands.none();
    }
    if (resetPose) {
      return runOnce(
              () -> {
                var startingPose = path.getStartingHolonomicPose().get();
                if (drivetrain.shouldFlipPath()) {
                  startingPose = FlippingUtil.flipFieldPose(startingPose);
                }
                drivetrain.setPose(startingPose);
              })
          .andThen(AutoBuilder.followPath(path));
    }
    return AutoBuilder.followPath(path);
  }

  public Command examplePath() {
    return runPath("examplepath", true);
  }
}
