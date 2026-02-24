package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

/**
 * Simple pneumatic subsystem with two double-solenoids (A and B).
 *
 * Channels are defined in `Constants.PneumaticConstants`.
 */
public class PneumaticSubsystem extends SubsystemBase {
  private final DoubleSolenoid m_solA = new DoubleSolenoid(
      PneumaticConstants.kPcmId,
      PneumaticsModuleType.CTREPCM,
      PneumaticConstants.kSolenoidAForward,
      PneumaticConstants.kSolenoidAReverse);

  private final DoubleSolenoid m_solB = new DoubleSolenoid(
      PneumaticConstants.kPcmId,
      PneumaticsModuleType.CTREPCM,
      PneumaticConstants.kSolenoidBForward,
      PneumaticConstants.kSolenoidBReverse);

  public PneumaticSubsystem() {}

  // --- Solenoid A ---
  public void setSolenoidAForward() {
    m_solA.set(Value.kForward);
  }

  public void setSolenoidAReverse() {
    m_solA.set(Value.kReverse);
  }

  public void setSolenoidAOff() {
    m_solA.set(Value.kOff);
  }

  public void toggleSolenoidA() {
    Value v = m_solA.get();
    if (v == Value.kForward) {
      m_solA.set(Value.kReverse);
    } else {
      m_solA.set(Value.kForward);
    }
  }

  // --- Solenoid B ---
  public void setSolenoidBForward() {
    m_solB.set(Value.kForward);
  }

  public void setSolenoidBReverse() {
    m_solB.set(Value.kReverse);
  }

  public void setSolenoidBOff() {
    m_solB.set(Value.kOff);
  }

  public void toggleSolenoidB() {
    Value v = m_solB.get();
    if (v == Value.kForward) {
      m_solB.set(Value.kReverse);
    } else {
      m_solB.set(Value.kForward);
    }
  }

  /** Toggle both solenoids together (forward <-> reverse). */
  public void toggleBoth() {
    // If both are forward, set both reverse; otherwise set both forward
    Value a = m_solA.get();
    Value b = m_solB.get();
    if (a == Value.kForward && b == Value.kForward) {
      m_solA.set(Value.kReverse);
      m_solB.set(Value.kReverse);
    } else {
      m_solA.set(Value.kForward);
      m_solB.set(Value.kForward);
    }
  }

  // --- Command factories ---
  public Command extendAShort() {
    return new InstantCommand(this::setSolenoidAForward, this);
  }

  public Command retractAShort() {
    return new InstantCommand(this::setSolenoidAReverse, this);
  }

  public Command toggleACommand() {
    return new InstantCommand(this::toggleSolenoidA, this);
  }

  public Command extendBShort() {
    return new InstantCommand(this::setSolenoidBForward, this);
  }

  public Command retractBShort() {
    return new InstantCommand(this::setSolenoidBReverse, this);
  }

  public Command toggleBCommand() {
    return new InstantCommand(this::toggleSolenoidB, this);
  }

  public Command toggleBothCommand() {
    return new InstantCommand(this::toggleBoth, this);
  }

  /**
   * Set both solenoids to forward.
   */
  public Command extendBoth() {
    return new InstantCommand(() -> {
      setSolenoidAForward();
      setSolenoidBForward();
    }, this);
  }

  /**
   * Set both solenoids to reverse.
   */
  public Command retractBoth() {
    return new InstantCommand(() -> {
      setSolenoidAReverse();
      setSolenoidBReverse();
    }, this);
  }
}
