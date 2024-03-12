package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import edu.wpi.first.wpilibj2.command.Command;

public class SetArmPositionCmd extends Command {
  private final ArmSubsystem arm;
  double targetShoulderPosition;
  double targetWristPosition;
  double elevatorPosition;
  ArmPosition target;

  boolean shoulderSetCheck = false;
  boolean wristSetCheck = false;
  boolean elevatorSetCheck;

  public SetArmPositionCmd(ArmSubsystem arm, ArmPosition target) {
    this.arm = arm;
    this.target = target;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
      arm.unsafeSetPosition(target);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    arm.unsafeSetPosition(ArmPosition.Stowed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}