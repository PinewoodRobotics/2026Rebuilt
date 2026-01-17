package frc.robot.command.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.TurretSubsystem;
import pwrup.frc.core.controller.FlightStick;

public class TurnTesting extends Command {

    private TurretSubsystem m_subsystem;
    private FlightStick fl;

    public TurnTesting(FlightStick fl) {
        this.m_subsystem = TurretSubsystem.GetInstance();
        this.fl = fl;

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double axis = fl.getRawAxis(FlightStick.AxisEnum.JOYSTICKROTATION.value);
        axis = MathUtil.applyDeadband(axis, 0.05);
        double targetRad = axis * Math.PI;

        m_subsystem.setTurretPosition(
                Units.Radians.of(targetRad),
                Units.Volts.of(2));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
