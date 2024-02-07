package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve m_swerve;    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;
    private BooleanSupplier m_robotCentricSup;

    public TeleopSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, Swerve swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);

        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
        m_robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        m_swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !m_robotCentricSup.getAsBoolean(), 
            true
        );
    }
}