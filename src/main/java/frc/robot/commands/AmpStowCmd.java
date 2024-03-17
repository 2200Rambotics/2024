package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpStowCmd extends Command {
  private final ArmSubsystem arm;
  boolean isFinished = false;

  public AmpStowCmd(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
      arm.unsafeSetPosition(ArmPosition.AmpStow);
      isFinished = false;
  }

  @Override
  public void execute() {
    if(arm.armAtPostion()){
      isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.unsafeSetPosition(ArmPosition.Stowed);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}