package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.AutoConstants;

/**
 * Commande qui crée un pathfinding vers une pose cible en utilisant AutoBuilder.
 */
public class DriveToPoseCommand extends ProxyCommand {

    public DriveToPoseCommand(Pose2d targetPose) {
        super(() -> AutoBuilder.pathfindToPose(
                targetPose,
                new PathConstraints(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        Units.degreesToRadians(540),
                        Units.degreesToRadians(720)),
                0.0));
    }
}
