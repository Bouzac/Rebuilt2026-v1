package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RamasseurSubsystem extends SubsystemBase{
    private final SparkFlex shaftInterieur;
    private final SparkFlex shaftExterieur;

    private final SparkClosedLoopController interieurPidController;
    private final SparkClosedLoopController exterieurPidController;

    private final SparkFlexConfig interieurConfig;
    private final SparkFlexConfig exterieurConfig;

    public RamasseurSubsystem(){
        shaftInterieur = new SparkFlex(9, MotorType.kBrushless);
        shaftExterieur = new SparkFlex(10, MotorType.kBrushless);

        interieurPidController = shaftInterieur.getClosedLoopController();
        exterieurPidController = shaftExterieur.getClosedLoopController();

        interieurConfig = new SparkFlexConfig();
        exterieurConfig = new SparkFlexConfig();

        interieurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.0001)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.00016);

        interieurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        exterieurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.0001)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.00016);

        exterieurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);
        shaftInterieur.configure(interieurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shaftExterieur.configure(exterieurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ramasserMode(){
        interieurPidController.setSetpoint(-1500, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
        exterieurPidController.setSetpoint(-1500, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    public void sortirMode(){
        interieurPidController.setSetpoint(1500, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
        exterieurPidController.setSetpoint(1500, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    public void lancerMode(){
        interieurPidController.setSetpoint(1500, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
        exterieurPidController.setSetpoint(-1500, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    public void stop(){
        shaftInterieur.set(0);
        shaftExterieur.set(0);
    }
}