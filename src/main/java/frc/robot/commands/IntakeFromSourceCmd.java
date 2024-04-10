package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFromSourceCmd extends Command {
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;
  public double speed;

  public IntakeFromSourceCmd(ArmSubsystem arm, ShooterSubsystem shooter, double speed) {
    this.shooter = shooter;
    this.speed = -speed;
    this.arm = arm;
    addRequirements(shooter);
  }
  
  @Override
  public void initialize() {
    arm.unsafeSetPosition(ArmPosition.Source);
    shooter.shooterState = ShooterState.SpinFixed;
    shooter.slowDownShooters = false;
    shooter.shooterV = speed;
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    shooter.shooterState = ShooterState.Idle;
    shooter.spinDownShooters();
    // shooter.shooterV = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
