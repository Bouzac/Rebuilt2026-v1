// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

/**
 * Système de conduite swerve refactoré : noms et commentaires en français,
 * structure plus claire, initialisation explicite du pose estimator.
 */
public class DriveSubsystem extends SubsystemBase {

	// ----- Modules swerve -----
	private final MAXSwerveModule avantGauche = new MAXSwerveModule(
			DriveConstants.kFrontLeftDrivingCanId,
			DriveConstants.kFrontLeftTurningCanId,
			DriveConstants.kFrontLeftChassisAngularOffset);

	private final MAXSwerveModule avantDroite = new MAXSwerveModule(
			DriveConstants.kFrontRightDrivingCanId,
			DriveConstants.kFrontRightTurningCanId,
			DriveConstants.kFrontRightChassisAngularOffset);

	private final MAXSwerveModule arriereGauche = new MAXSwerveModule(
			DriveConstants.kRearLeftDrivingCanId,
			DriveConstants.kRearLeftTurningCanId,
			DriveConstants.kBackLeftChassisAngularOffset);

	private final MAXSwerveModule arriereDroite = new MAXSwerveModule(
			DriveConstants.kRearRightDrivingCanId,
			DriveConstants.kRearRightTurningCanId,
			DriveConstants.kBackRightChassisAngularOffset);

	// ----- Gyro -----
	private final GyroIO m_gyro = new GyroIO();

	// ----- Pose estimator (initialisé dans le constructeur) -----
	private SwerveDrivePoseEstimator poseEstimator;

	// ----- Affichage sur le terrain -----
	private final Field2d field2d = new Field2d();

	// ----- Contrôleurs PID pour conduite vers une pose -----
	private final PIDController xController = new PIDController(1.5, 0.0, 0.0); // m/s per m
	private final PIDController yController = new PIDController(1.5, 0.0, 0.0); // m/s per m
	private final PIDController thetaController = new PIDController(3.0, 0.0, 0.0); // deg/s per deg

	// ----- Tolérances -----
	private final double positionToleranceMeters = 0.05; // 5 cm
	private final double angleToleranceDegrees = 3.0; // 3 degrees

	public DriveSubsystem() {
		// Initialisation des encodeurs / odométrie
		resetEncoders();

		// Initialiser le poseEstimator maintenant que le gyro et les modules existent
		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() },
				Pose2d.kZero);

		// Réinitialiser l'odométrie à l'origine par défaut
		resetOdometry(new Pose2d());

		// Configurer les PID
		thetaController.enableContinuousInput(-180.0, 180.0);
		xController.setTolerance(positionToleranceMeters);
		yController.setTolerance(positionToleranceMeters);
		thetaController.setTolerance(angleToleranceDegrees);

		// Register field on SmartDashboard
		SmartDashboard.putData("Field", field2d);
	}

	@Override
	public void periodic() {
		// Mettre à jour l'estimation de pose avec les positions de modules
		poseEstimator.update(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() });

		// Affichage pour debug
		SmartDashboard.putNumber("Angle Gyro", getAngle());
		SmartDashboard.putNumber("Pose Estimator X", getPose().getX());
		SmartDashboard.putNumber("Pose Estimator Y", getPose().getY());
		SmartDashboard.putNumber("Pose Estimator Theta", getPose().getRotation().getDegrees());

		// Intégration Limelight (position/orientation)
		setLimelightRobotOrientation();
		addVisionPosition("limelight");

		// Mettre à jour la vue Field2d
		field2d.setRobotPose(getPose());
	}

	// ----- Commande des modules -----

	/**
	 * Envoie les états désirés à chaque module.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		avantGauche.setDesiredState(desiredStates[0]);
		avantDroite.setDesiredState(desiredStates[1]);
		arriereGauche.setDesiredState(desiredStates[2]);
		arriereDroite.setDesiredState(desiredStates[3]);
	}

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] { avantGauche.getState(), avantDroite.getState(),
				arriereGauche.getState(), arriereDroite.getState() };
	}

	/**
	 * Conduite manuelle : inputs normalisés [-1,1]. fieldRelative indique si les
	 * vitesses sont en repère terrain.
	 */
	public void conduire(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean squared) {
		if (squared) {
			xSpeed = xSpeed * Math.abs(xSpeed);
			ySpeed = ySpeed * Math.abs(ySpeed);
			rot = rot * Math.abs(rot);
		}

		double xSpeedMeters = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double ySpeedMeters = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double rotRad = rot * DriveConstants.kMaxAngularSpeed;

		// Inversion seulement si field-relative
		double invert = (fieldRelative && isRedAlliance()) ? -1.0 : 1.0;

		ChassisSpeeds speeds = fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMeters * invert, ySpeedMeters * invert, rotRad,
						getPose().getRotation())
				: new ChassisSpeeds(xSpeedMeters, ySpeedMeters, rotRad);

		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
	}

	public void stop() {
		// Mettre des vitesses nulles aux moteurs
		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
	}

	/**
	 * Positionner les roues en X pour bloquer le robot.
	 */
	public void setX() {
		avantGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		avantDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		arriereGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		arriereDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	// ----- Pose estimator / odométrie -----

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetPose(Pose2d pose) {
		resetOdometry(pose);
	}

	public void resetOdometry(Pose2d pose) {
		poseEstimator.resetPosition(Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() },
				pose);
	}

	// ----- Limelight helpers -----

	public void setLimelightRobotOrientation() {
		LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
	}

	public void addVisionPosition(String nomComplet) {
		// Paramètres de confiance pour la mesure vision
		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

		LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(nomComplet);
		if (poseEstimate == null) {
			return;
		}

		boolean doRejectUpdate = false;
		if (Math.abs(getRate()) > 720) {
			doRejectUpdate = true;
		}
		if (poseEstimate.tagCount == 0) {
			doRejectUpdate = true;
		}
		SmartDashboard.putBoolean(nomComplet, !doRejectUpdate);
		if (!doRejectUpdate) {
			poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
		}
	}

	public void setZeroPosition() {
		resetOdometry(new Pose2d());
	}

	// ----- Encodeurs -----

	public void resetEncoders() {
		avantGauche.resetEncoders();
		arriereGauche.resetEncoders();
		avantDroite.resetEncoders();
		arriereDroite.resetEncoders();
	}

	// ----- Gyro -----

	public double getAngle() {
		// Retourne l'angle en degrés (convention négative pour ce projet)
		return -m_gyro.getYaw();
	}

	public double getRate() {
		return m_gyro.getRate();
	}

	public void resetGyro() {
		m_gyro.reset();
	}

	// ----- Path planner helpers -----

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(
				avantDroite.getState(),
				avantGauche.getState(),
				arriereDroite.getState(),
				arriereGauche.getState());
	}

	public void conduireChassis(ChassisSpeeds chassisSpeeds) {
		// Discrétiser pour cadence de 20 ms (contrôleurs périodiques)
		ChassisSpeeds target = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
		SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(target);
		setModuleStates(states);
	}

	// ----- Alliance -----

	/**
	 * Vérifie si l'alliance est rouge. Appeler souvent car l'alliance peut être
	 * initialisée après le boot.
	 */
	public boolean isRedAlliance() {
		Optional<Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent()) {
			return ally.get() == Alliance.Red;

		} else {
			return false;
		}
	}

	@Override
    public void simulationPeriodic() {
        // Calcul théorique
        ChassisSpeeds robotSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        double angularVelocityDeg = Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond);
        double angleChange = angularVelocityDeg * 0.02;

		avantGauche.simulationPeriodic(0.02);
		avantDroite.simulationPeriodic(0.02);
		arriereGauche.simulationPeriodic(0.02);
		arriereDroite.simulationPeriodic(0.02);

        // Mise à jour via le Wrapper
        // On récupère l'ancien angle simulé via getAngle(), on ajoute le delta
        // Attention aux signes : getAngle() retourne déjà l'inverse (-yaw) dans ta logique
        double currentSimAngle = m_gyro.getYaw(); 
        
        // Logique simplifiée pour la sim :
        m_gyro.setSimYaw(currentSimAngle + angleChange); // ou -angleChange selon ta convention CCW/CW
    }
}