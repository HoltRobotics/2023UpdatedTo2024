package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.Arm.DownArm;
import frc.robot.commands.Arm.UpArm;
import frc.robot.commands.Combo.DropStage;
import frc.robot.commands.Combo.FloorHorizontal;
import frc.robot.commands.Combo.FloorVertical;
import frc.robot.commands.Combo.SlideStage;
import frc.robot.commands.Combo.Stage1;
import frc.robot.commands.Combo.Stage2;
import frc.robot.commands.Combo.Stage3;
import frc.robot.commands.Combo.StowArm;
import frc.robot.commands.Elevator.Down;
import frc.robot.commands.Elevator.LowerCone;
import frc.robot.commands.Elevator.RaiseCone;
import frc.robot.commands.Elevator.Up;
import frc.robot.commands.Pneumatics.BuddyDown;
import frc.robot.commands.Pneumatics.BuddyUp;
import frc.robot.commands.Pneumatics.CloseClaw;
import frc.robot.commands.Pneumatics.OpenClaw;
import frc.robot.commands.Pneumatics.ToggleTilt;
import frc.robot.commands.Swerve.Balance;
import frc.robot.commands.Swerve.FastBalance;
import frc.robot.commands.Swerve.ResetEncoders;
import frc.robot.commands.Swerve.SlowDrive;
import frc.robot.commands.Swerve.ZeroGyro;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final PS5Controller m_driver = new PS5Controller(Constants.kDriverPort);
    private final Joystick m_operator = new Joystick(Constants.kOperatorPort);

    /* Subsystems */
    private final Swerve m_swerve = new Swerve();
    private final ArmProfiled m_arm = new ArmProfiled();
    private final ElevatorProfiled m_lift = new ElevatorProfiled();
    private final Pneumatics m_air = new Pneumatics();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                () -> -m_driver.getLeftY(),     // Translation
                () -> -m_driver.getLeftX(),     // Strafe
                () -> -m_driver.getRightX(),    // Rotation
                () -> m_driver.getCircleButton(), // Field Centric
                m_swerve
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new POVButton(m_driver, 180).whileTrue(new Down(m_lift));
        new POVButton(m_driver, 0).whileTrue(new Up(m_lift));
        new POVButton(m_driver, 90).whileTrue(new DownArm(m_arm));
        new POVButton(m_driver, 270).whileTrue(new UpArm(m_arm));

        new JoystickButton(m_driver, 6).toggleOnFalse(new SlowDrive(m_swerve));
        new JoystickButton(m_driver, 1).onTrue(new ToggleTilt(m_air));
        new JoystickButton(m_driver, 14).onTrue(new ZeroGyro(m_swerve));
        new JoystickButton(m_driver, 9).onTrue(new ResetEncoders(m_swerve));
        new JoystickButton(m_driver, 2).whileTrue(new OpenClaw(m_air)).whileFalse(new CloseClaw(m_air));

        new JoystickButton(m_operator, 5).whileTrue(new SequentialCommandGroup(new FastBalance(m_swerve), new Balance(m_swerve)));

        new JoystickButton(m_operator, 2).onTrue(new StowArm(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 1).onTrue(new SlideStage(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 3).onTrue(new DropStage(m_arm, m_lift, m_air, m_swerve));
        // new JoystickButton(m_operator, 5).onTrue(new ClawUp(m_air));
        new JoystickButton(m_operator, 11).onTrue(new Stage1(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 13).onTrue(new Stage2(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 15).onTrue(new Stage3(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 12).onTrue(new FloorHorizontal(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 14).onTrue(new FloorVertical(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 4).onTrue(new RaiseCone(m_lift));
        new JoystickButton(m_operator, 9).onTrue(new LowerCone(m_lift));
        new JoystickButton(m_operator, 10).onTrue(new BuddyDown(m_air)).onFalse(new BuddyUp(m_air));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new WaitCommand(15);
    }
}
