package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class KeybindTestCmd extends Command {
  private final LEDSubsystem led;

  public int stripNum;

  public KeybindTestCmd(LEDSubsystem led, int stripNum) {
    this.led = led;
    this.stripNum = stripNum;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch(stripNum){
      case 1: led.setRed(); break;
      case 2: led.setBlue(); break;
      case 3: led.setYellow(); break;
      case 4: led.setGreen(); break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    switch(stripNum){
      case 1: led.setColour(Color.kBlack, led.showingBuffer, led.quadStrip1); break;
      case 2: led.setColour(Color.kBlack, led.showingBuffer, led.quadStrip2); break;
      case 3: led.setColour(Color.kBlack, led.showingBuffer, led.quadStrip3); break;
      case 4: led.setColour(Color.kBlack, led.showingBuffer, led.quadStrip4); break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
