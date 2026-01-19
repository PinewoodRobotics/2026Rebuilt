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
        axis = MathUtil.clamp(axis, 0.0, 1.0);
        m_subsystem.setTurretPosition(
                Units.Rotations.of(axis),
                Units.Volts.of(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
