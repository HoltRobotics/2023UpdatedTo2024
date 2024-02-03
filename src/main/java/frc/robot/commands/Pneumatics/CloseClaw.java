// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Pneumatics;

public class CloseClaw extends InstantCommand {
  private final Pneumatics m_air; // The subsystem needed to control the Pneumatics.

  /**
   * Command that closes the claw.
   * @param air The Pneumatics Subsystem
   */
  public CloseClaw(Pneumatics air) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_air = air; // Passes the given subsystem to the rest of the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_air.setClawState(false); // Closes the claw.
  }
}
