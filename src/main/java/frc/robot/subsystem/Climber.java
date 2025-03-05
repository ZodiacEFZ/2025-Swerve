package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonSRXMotor;

public class Climber extends SubsystemBase {
    private final TalonSRXMotor climberMotor = new TalonSRXMotor(30);
    private Position position = Position.DOWN;

    public Climber() {
        this.climberMotor.factoryDefault();
        this.climberMotor.setSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.climberMotor.setPhase(false);
        this.climberMotor.setEncoderZero(-942);
        this.climberMotor.setInverted(true);
        var climberPID = new PIDController(0.5, 0.001, 0.01);
        climberPID.setIZone(200);
        this.climberMotor.setPID(climberPID);
        this.climberMotor.setBrakeWhenNeutral(true);
        this.climberMotor.setSoftwareLimitSwitch(Position.DOWN.position, Position.CLIMB.position);
    }

    public Command getDownCommand() {
        return runOnce(this::down);
    }

    public void down() {
        this.climberMotor.setPosition(Position.DOWN.position);
        this.position = Position.DOWN;
    }

    public Command getUpCommand() {
        return runOnce(this::up);
    }

    public void up() {
        this.climberMotor.setPosition(Position.UP.position);
        this.position = Position.UP;
    }

    public Command getClimbCommand() {
        return runOnce(this::climb);
    }

    public void climb() {
        this.climberMotor.setPosition(Position.CLIMB.position);
        this.position = Position.CLIMB;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climber");
        builder.setActuator(true);
        builder.addDoubleProperty("Position", () -> this.getPosition().in(Units.Radians), null);
    }

    private Angle getPosition() {
        return this.climberMotor.getPosition();
    }

    public Command getSwitchClimberStateCommand() {
        return runOnce(this::switchClimberState);
    }

    private void switchClimberState() {
        if (this.position == Position.UP) {
            this.down();
        } else {
            this.up();
        }
    }

    enum Position {
        DOWN(0),
        UP(1.4),
        CLIMB(4);

        final Angle position;

        Position(double rad) {
            this.position = Units.Radians.of(rad);
        }
    }
}

