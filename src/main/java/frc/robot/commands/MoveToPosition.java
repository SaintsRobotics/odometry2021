/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.*;

public class MoveToPosition extends CommandBase {
  /**
   * Creates a new MoveToPosition.
   */
  private SwerveDrivetrain m_drivetrain;
  private PIDController m_controller;
  private Pose2D m_currentPose;

  public MoveToPosition(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    x_PIDcontroller = new PIDController(.3, 0, 0);
    m_currentPose = m_drivetrain.getLocation();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // m_drivetrain.move(x, y, a, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.move(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
