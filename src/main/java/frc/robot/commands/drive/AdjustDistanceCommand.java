package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Commande pour avancer/reculer jusqu'à une distance cible (pose X)
 * en maintenant un offset par rapport à la consigne reçue.
 */
public class AdjustDistanceCommand extends FunctionalCommand {

    public AdjustDistanceCommand(DriveSubsystem drive, double targetDistance) {
        super(
            () -> {
                drive.getXController().reset();
                // Rester 2 unités plus loin que la pose demandée
                drive.getXController().setSetpoint(targetDistance);
            },
            () -> {
                double currentX = drive.getPose().getX();
                double pidOutput = drive.getXController().calculate(currentX);

                // Limiter la vitesse translative pour rester doux
                double xInput = MathUtil.clamp(pidOutput, -0.6, 0.6);
                drive.conduire(xInput, 0, 0, true, false);
            },
            (interrupted) -> drive.stop(),
            () -> drive.getXController().atSetpoint(),
            drive
        );
    }
}
