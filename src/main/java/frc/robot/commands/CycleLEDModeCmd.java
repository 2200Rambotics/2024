package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CycleLEDModeCmd extends Command {
  private final LEDSubsystem led;
  private boolean isFinished = false;

  public CycleLEDModeCmd(LEDSubsystem led) {
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    led.disabledMode++;
    if(led.disabledMode == led.disabledModes){
      led.disabledMode = 0;
    }
    isFinished = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
