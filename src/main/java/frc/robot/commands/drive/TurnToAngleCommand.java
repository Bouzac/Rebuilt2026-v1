package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Commande "one-shot" pour orienter le robot vers un angle cible.
 * Elle réutilise le PID theta interne du {@link DriveSubsystem} pour garder
 * les mêmes constantes et tolérances.
 */
public class TurnToAngleCommand extends FunctionalCommand {

    public TurnToAngleCommand(DriveSubsystem drive, Rotation2d targetAngle) {
        super(
            () -> {
                drive.getThetaController().reset();
                drive.getThetaController().setSetpoint(targetAngle.getDegrees());
            },
            () -> {
                double currentAngle = drive.getPose().getRotation().getDegrees();

                double pidOutput = drive.getThetaController().calculate(currentAngle, targetAngle.getDegrees());
                pidOutput += Math.signum(pidOutput) * 0.05; // petit feedforward pour lancer le mouvement

                double rotationInput = MathUtil.clamp(pidOutput, -0.5, 0.5);
                drive.conduire(0, 0, rotationInput, false, false);
            },
            (interrupted) -> drive.stop(),
            () -> drive.getThetaController().atSetpoint(),
            drive
        );
    }
}
