package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SourceShotCmd extends Command {
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  double speed;
  Timer shootTimer;

  public SourceShotCmd(ArmSubsystem arm, ShooterSubsystem shooter, double speed) {
    this.arm = arm;
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(arm, shooter);
  }

  @Override
  public void initialize() {
    arm.unsafeSetPosition(ArmPosition.SourceShot);
    shootTimer = new Timer();
    shootTimer.start();
    shooter.intakeState = IntakeState.ResetTimer;
  }

  @Override
  public void execute() {
    if(shootTimer.get() > 0.05 && shootTimer.get() < 0.1){
      shooter.intakeState = IntakeState.Preload;
    }
    if(!shooter.slowDownShooters){
      shooter.shooterV = speed;
      shooter.shooterState = ShooterState.SpinFixed;
    }
    if(shootTimer.get() > 0.8){
      shooter.intakeState = IntakeState.ShootNow;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.shooterState = ShooterState.Idle;
    shooter.spinDownShooters();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
