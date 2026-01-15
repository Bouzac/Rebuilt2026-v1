package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AutoConstants;
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
	private final PIDController thetaController = new PIDController(0.015, 0.0, 0.001); // deg/s per deg

	// ----- Tolérances -----
	private final double positionToleranceMeters = 0.05; // 5 cm
	private final double angleToleranceDegrees = 0.5; // 0.01 degrees

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

		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (IOException | ParseException e) {
			throw new RuntimeException("Failed to load RobotConfig from GUI settings", e);
		}

		AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> conduireChassis(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(1.5, 0.0, 0.0),
                    new PIDConstants(6, 0.0, 0.05)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
    	);

		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			field2d.getObject("activePath").setPoses(poses);
		});
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
		double normalizedAngle = (m_gyro.getAngle() + 180) % 360;

		if (normalizedAngle <= 0) {
			normalizedAngle += 360;
		}

		// Retourne l'angle en degrés (convention négative pour ce projet)
		return normalizedAngle - 180;
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
				avantGauche.getState(),
				avantDroite.getState(),
				arriereGauche.getState(),
				arriereDroite.getState());
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
        double currentSimAngle = m_gyro.getAngle(); 
        
        // Logique simplifiée pour la sim :
        m_gyro.setSimYaw(currentSimAngle + angleChange); // ou -angleChange selon ta convention CCW/CW
    }

	public Command driveToPoseCommand(Pose2d targetPose) {
		PathConstraints constraints = new PathConstraints(
				AutoConstants.kMaxSpeedMetersPerSecond, 
				AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
				Units.degreesToRadians(540), 
				Units.degreesToRadians(720));

		return AutoBuilder.pathfindToPose(
				targetPose,
				constraints,
				0.0 
		);
	}
	

	public Rotation2d getAngleToBasket() {
		double robotX = getPose().getX();
		double robotY = getPose().getY();

		double basketX; 
		double basketY = 4.0; 

		boolean isRed = isRedAlliance();
		basketX = isRed ? 5.0 : 12.5;

		boolean isOnGoodSide = false;
		if (isRed) {
			if (robotX < basketX - 0.2) isOnGoodSide = true;
		} else {
			if (robotX > basketX + 0.2) isOnGoodSide = true;
		}

		if (isOnGoodSide) {
			double deltaX = basketX - robotX;
			double deltaY = basketY - robotY;

			double angleRad = Math.atan2(deltaY, deltaX);

			return new Rotation2d(angleRad);
		} else {
			return getPose().getRotation();
		}
	}

	/**
	 * Calcule une pose située à une distance donnée du panier, sur la ligne
	 * panier -> robot, en orientant le robot vers le panier.
	 * @param distanceMeters distance souhaitée entre le robot et le panier
	 * @return Pose2d cible à cette distance, orientée vers le panier
	 */
	public Pose2d getPoseFacingBasketAtDistance(double distanceMeters) {
		double robotX = getPose().getX();
		double robotY = getPose().getY();

		double basketX;
		double basketY = 4.0;
		boolean isRed = isRedAlliance();
		basketX = isRed ? 5.0 : 12.5;

		double dx = robotX - basketX;
		double dy = robotY - basketY;
		double currentDist = Math.hypot(dx, dy);

		// Sécurité pour éviter division par zéro
		if (currentDist < 1e-3) {
			currentDist = 1e-3;
		}

		// Positionner le robot sur le cercle à distance demandée
		double scale = distanceMeters / currentDist;
		double targetX = basketX + dx * scale;
		double targetY = basketY + dy * scale;

		Rotation2d targetHeading = new Rotation2d(Math.atan2(basketY - targetY, basketX - targetX));
		return new Pose2d(targetX, targetY, targetHeading);
	}

	public Command turnToAngleCommand(Rotation2d targetAngle) {
		return new FunctionalCommand(
			() -> {
				thetaController.reset();
				thetaController.setSetpoint(targetAngle.getDegrees());
			},
			() -> {
				double currentAngle = getPose().getRotation().getDegrees();

				double pidOutput = thetaController.calculate(currentAngle, targetAngle.getDegrees());

				pidOutput += Math.signum(pidOutput) * 0.05; 

				// Clamp critique : on force la valeur entre -0.5 et 0.5 (50% de Vmax)
				// Cela empêche le robot de recevoir une demande de 400% de vitesse.
				double rotationInput = MathUtil.clamp(pidOutput, -0.5, 0.5);

				conduire(0, 0, rotationInput, false, false);
			},
			(interrupted) -> stop(),
			() -> thetaController.atSetpoint(),
			this
		);
	}

	public double getCompensationRotation(Double targetAngleDegrees) {
		double currentAngle = getPose().getRotation().getDegrees();

		double pidOutput = thetaController.calculate(currentAngle, targetAngleDegrees);

		pidOutput += Math.signum(pidOutput) * 0.05; 

		// Clamp critique : on force la valeur entre -0.5 et 0.5 (50% de Vmax)
		// Cela empêche le robot de recevoir une demande de 400% de vitesse.
		double rotationInput = MathUtil.clamp(pidOutput, -0.5, 0.5);

		return rotationInput;
	}

	/**
	 * Accès au contrôleur d'angle pour les commandes externes.
	 */
	public PIDController getThetaController() {
		return thetaController;
	}

	public PIDController getXController() {
		return xController;
	}
}