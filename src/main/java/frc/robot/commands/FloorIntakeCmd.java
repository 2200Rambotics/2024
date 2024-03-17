package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem.FloorIntakeState;

public class FloorIntakeCmd extends Command {
  private final FloorIntakeSubsystem floorIntake;
  FloorIntakeState state;
  double waitTime;
  Timer timer;

  public FloorIntakeCmd(FloorIntakeSubsystem floorIntake, FloorIntakeState state, double waitTime) {
    this.floorIntake = floorIntake;
    this.state = state;
    this.waitTime = waitTime;
    timer = new Timer();
    addRequirements(floorIntake);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    if(timer.get() >= waitTime){
      floorIntake.set(state);
    }
  }

  @Override
  public void end(boolean interrupted) {
    floorIntake.set(FloorIntakeState.Stop);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
