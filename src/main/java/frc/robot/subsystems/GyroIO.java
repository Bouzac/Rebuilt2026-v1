package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Wrapper autour du capteur NavX (AHRS) qui gère l'utilisation réelle sur
 * le robot et une simulation simple en mode standalone.
 *
 * Comportement :
 * - En mode réel : initialise le capteur NavX via MXP SPI. En cas d'échec,
 *   une exception est interceptée et l'instance reste à null.
 * - En simulation : l'instance matérielle reste null et un angle simulé est
 *   utilisé via setSimYaw(double).
 */
public class GyroIO {
    // Instance du capteur réel ; null si non disponible (simulation ou erreur d'initialisation).
    private AHRS m_realGyro;

    // Angle yaw utilisé uniquement en simulation.
    private double m_simYaw;

    /**
     * Initialise le sous-système gyro. Sur le robot réel, tente d'ouvrir
     * l'interface NavX via MXP SPI. En simulation, aucun matériel n'est ouvert.
     */
    public GyroIO() {
        if (RobotBase.isReal()) {
            try {
                m_realGyro = new AHRS(NavXComType.kMXP_SPI);
            } catch (RuntimeException e) {
                // Ne pas propager l'exception pour éviter d'empêcher le démarrage du code.
                // L'utilisateur/gestionnaire supérieur peut vérifier que l'instance est null.
                System.out.println("NavX initialization error: " + e.getMessage());
            }
        } else {
            // En simulation, le capteur réel n'est pas initialisé.
            System.out.println("Simulation mode: NavX not initialized.");
        }
    }

    /**
     * Retourne l'angle yaw actuel en degrés.
     * Note : l'inversion du signe (-) est appliquée pour correspondre à la convention
     * de l'application (sens de rotation attendu).
     *
     * @return yaw en degrés (simulé si pas de capteur réel)
     */
    public double getYaw() {
        if (m_realGyro != null) {
            return -m_realGyro.getYaw();
        }
        return m_simYaw;
    }

    /**
     * Retourne la vitesse angulaire actuelle (deg/s).
     * En simulation basique, retourne 0.0 si aucun capteur réel n'est présent.
     *
     * @return taux de rotation en deg/s ou 0.0 en mode simulation par défaut
     */
    public double getRate() {
        if (m_realGyro != null) {
            return m_realGyro.getRate();
        }
        return 0.0;
    }

    /**
     * Réinitialise l'angle du gyro.
     * Sur matériel réel, délègue à l'AHRS ; en simulation, remet l'angle simulé à 0.
     */
    public void reset() {
        if (m_realGyro != null) {
            m_realGyro.reset();
        }
        m_simYaw = 0.0;
    }

    /**
     * Méthode destinée à la simulation : injecte un angle yaw simulé.
     * Ne modifie aucun état matériel.
     *
     * @param yaw angle simulé en degrés
     */
    public void setSimYaw(double yaw) {
        this.m_simYaw = yaw;
    }
}