package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.DriveConstants;

public class ConveyorSubsystem extends SubsystemBase {
  private final SparkMax m_motor; 

  public ConveyorSubsystem() {
    m_motor = new SparkMax(ConveyorConstants.kConveyorSparkID, MotorType.kBrushless);
    configureMotor();
  }

  private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(30);
    
    // Config'i uygula
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runConveyor() {
    m_motor.setVoltage(12);
  }

  public void runConveyor(double speed) {
    // Uygulanan voltajı ±kMaxMotorVoltage olacak şekilde ayarla (speed işareti yönü belirler)
    double applied = Math.copySign(DriveConstants.kMaxMotorVoltage, speed);
    m_motor.setVoltage(applied);
  }
  public void stopConveyor() {
    m_motor.setVoltage(0);
  }

  public Command runConveyorCommand( double speed) {
    return this.startEnd(
        () -> runConveyor(speed),
        () -> stopConveyor()
    );
  }

  public Command reverseConveyorCommand() {
    return this.startEnd(
        () -> runConveyor(-ConveyorConstants.kConveyorSpeed),
        () -> stopConveyor()
    );
  }
}
