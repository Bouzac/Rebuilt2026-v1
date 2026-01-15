// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drive.DriveToPoseCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    // Bouton 1 lance la commande fournie par le sous-système pour aller à une pose précise.
    Pose2d cible = new Pose2d(10.4, 3.8, new Rotation2d(Units.degreesToRadians(180)));
    new JoystickButton(m_driverController, 1)
        .whileTrue(new DriveToPoseCommand(cible));

    new JoystickButton(m_driverController, 2)
        .whileTrue(
            Commands.defer(
                () -> new TurnToAngleCommand(m_robotDrive, m_robotDrive.getAngleToBasket()),
                java.util.Set.of(m_robotDrive)));
  }
}
