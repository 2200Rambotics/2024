package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroArmCommand extends Command {
  private final ArmSubsystem arm;

  public ZeroArmCommand(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    arm.leftShoulderMotor.setPercentOutput(0.05);
    arm.wristMotor.setPercentOutput(-0.02);
    arm.elevatorMotor.setPercentOutput(0.04);
  }

  @Override
  public void end(boolean interrupted) {
    arm.leftShoulderMotor.setPercentOutput(0);
    arm.wristMotor.setPercentOutput(0);
    arm.elevatorMotor.setPercentOutput(0);
    arm.zeroEncoders();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
