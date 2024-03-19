package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SignalNoteCmd extends Command {
  private final LEDSubsystem led;

  public SignalNoteCmd(LEDSubsystem led) {
    this.led = led;
  }

  @Override
  public void initialize() {
    led.isSignaling = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    led.isSignaling = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
