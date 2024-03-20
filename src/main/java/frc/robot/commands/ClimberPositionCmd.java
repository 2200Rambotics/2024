package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPositionCmd extends Command {
  private final ClimberSubsystem climber;
  private final ArmSubsystem arm;
  public ClimbState state;
  public boolean isFinished = false;

  public ClimberPositionCmd(ClimberSubsystem climber, ArmSubsystem arm, ClimbState state) {
    this.climber = climber;
    this.arm = arm;
    this.state = state;
    addRequirements(climber, arm);
  }

  @Override
  public void initialize() {
    climber.setHeight(state.height());
    switch (state) {
      case Max: arm.unsafeSetPosition(ArmPosition.ClimberUp); break;
      case Mid: arm.unsafeSetPosition(ArmPosition.ClimberMid); break;
      case Min: arm.unsafeSetPosition(ArmPosition.ClimberLow); break;
      case Stowed: arm.unsafeSetPosition(ArmPosition.ClimberStowed); break;
      case Compact: arm.unsafeSetPosition(ArmPosition.ClimberCompact); break;
      default: break;
    }


    isFinished = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
      climber.disableMotors(state == ClimbState.Stowed);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}