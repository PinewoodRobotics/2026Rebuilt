package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.command.scoring.TurnTesting;
import frc.robot.subsystem.TurretSubsystem;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;
import pwrup.frc.core.controller.LogitechController;
import pwrup.frc.core.controller.OperatorPanel;

public class RobotContainer {

  final LogitechController m_controller = new LogitechController(0);
  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
      m_leftFlightStick,
      m_rightFlightStick);

  public RobotContainer() {
    TurretSubsystem.GetInstance();
    configureBindings();
  }

  private void configureBindings() {
    TurretSubsystem.GetInstance().setDefaultCommand(new TurnTesting(m_leftFlightStick));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void onAnyModeStart() {
  }
}
