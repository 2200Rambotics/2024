package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooterCmd extends Command {
  private final ShooterSubsystem shooter;
  public double speed;
  public boolean isAmp;

  public SpinUpShooterCmd(ShooterSubsystem shooter, double speed, boolean isAmp) {
    this.shooter = shooter;
    this.speed = speed;
    this.isAmp = isAmp;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.slowDownShooters = false;
    if(isAmp){
      shooter.shooterState = ShooterState.SpinAmp;
    } else{
      shooter.shooterState = ShooterState.SpinFixed;
    }
    shooter.shooterV = speed;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.shooterState = ShooterState.Idle;
    // shooter.shooterV = 0;
    shooter.spinDownShooters();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
