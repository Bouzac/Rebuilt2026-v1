// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drive.TurnToAngleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Conteneur principal du robot.
 * 
 * Cette classe déclare les sous-systèmes, la configuration des commandes par défaut
 * et les mappages des boutons. Les commentaires et noms sont en français pour
 * clarifier l'intention.
 */
public class RobotContainer {
  // Sous-système du châssis (drive)
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Manette du conducteur
  private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);

  /**
   * Constructeur : configure les bindings et la commande par défaut.
   */
  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Définit la commande par défaut du châssis : lecture des axes de la manette
   * et appel à la méthode "conduire" du sous-système.
   */
  private void configureDefaultCommands() {
    // Lecture des axes, application d'un deadband, et mise à l'échelle par les constantes de vitesse.
    // La méthode conduire(x, y, rot, fieldRelative, autreFlag) est appelée régulièrement.
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.conduire(
                    -MathUtil.applyDeadband(
                        m_driverController.getRawAxis(1) * DriveConstants.kVitesse,
                        OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRawAxis(0) * DriveConstants.kVitesse,
                        OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        -m_driverController.getRawAxis(4) * DriveConstants.kVitesseRotation,
                        OIConstants.kDriveDeadband),
                    true, // fieldRelative : true si contrôle relatif au terrain
                    false
                    ),
            m_robotDrive));
  }

  /**
   * Configure les mappages boutons -> commandes.
   * - Bouton 8 : remettre l'odométrie / position zéro du châssis.
   * - Bouton 1 : lancer la commande de déplacement vers une Pose fixe.
   */
  private void configureButtonBindings() {
    // Quand on appuie (onTrue) sur le bouton 8, exécuter instantanément setZeroPosition()
    new JoystickButton(m_driverController, 8)
        .onTrue(new InstantCommand(m_robotDrive::setZeroPosition, m_robotDrive));

    new JoystickButton(m_driverController, 2)
        .whileTrue(
            Commands.defer(
                () -> new TurnToAngleCommand(m_robotDrive, m_robotDrive.getAngleToBasket()),
                java.util.Set.of(m_robotDrive)));

    new Trigger(m_driverController::getLeftBumperButton)
        .whileTrue(new RunCommand(
            () ->
                {
                    // Vitesses demandées (field-relative)
                    double vx = MathUtil.applyDeadband(
                        m_driverController.getRawAxis(1) * DriveConstants.kVitesse,
                        OIConstants.kDriveDeadband);
                    double vy = MathUtil.applyDeadband(
                        m_driverController.getRawAxis(0) * DriveConstants.kVitesse,
                        OIConstants.kDriveDeadband);

                    // Centre du panier selon l'alliance
                    double basketY = 4.0;
                    double basketX = m_robotDrive.isRedAlliance() ? 5.0 : 12.5;

                    double px = m_robotDrive.getPose().getX();
                    double py = m_robotDrive.getPose().getY();

                    double dx = px - basketX;
                    double dy = py - basketY;
                    double dist = Math.hypot(dx, dy);
                    if (dist < 1e-3) dist = 1e-3; // éviter division par zéro

                    double nx = dx / dist;
                    double ny = dy / dist;

                    // Projection tangentielle : retirer la composante radiale
                    double radial = vx * nx + vy * ny;
                    double vxT = vx - radial * nx;
                    double vyT = vy - radial * ny;

                    // Correction douce pour rester à rayon 2 m
                    double radiusError = dist - 5.0;
                    double kR = 0.5; // gain radial
                    double radialCorr = -kR * radiusError;

                    double vxCmd = vxT + radialCorr * nx;
                    double vyCmd = vyT + radialCorr * ny;

                    // Clamp de sécurité
                    vxCmd = MathUtil.clamp(vxCmd, -1.0, 1.0);
                    vyCmd = MathUtil.clamp(vyCmd, -1.0, 1.0);

                    double rotCmd = m_robotDrive.getCompensationRotation(m_robotDrive.getAngleToBasket().getDegrees());

                    m_robotDrive.conduire(
                        -vxCmd,
                        -vyCmd,
                        rotCmd,
                        true, // fieldRelative : true si contrôle relatif au terrain
                        false
                        );
                },
            m_robotDrive));
  }
}
