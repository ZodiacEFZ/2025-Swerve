package frc.robot.subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;
import frc.libzodiac.util.Maths;

public class Climber extends SubsystemBase {
    private final MagEncoder encoder = new MagEncoder(30, -1675);
    private final TalonFXMotor motor = new TalonFXMotor(31);
    private Position position = Position.DOWN;

    public Climber() {
        this.encoder.setInverted(true);
        this.encoder.setContinuous(false);
        this.motor.factoryDefault();
        this.motor.setPID(new PIDController(75, 0, 0));
        this.motor.setBrakeWhenNeutral(true);
        this.motor.setSensorToMechanismRatio(58.0 / 18 * 100);
        var angle = Maths.limitAngle(this.encoder.get());
        this.motor.setRelativeEncoderPosition(Maths.limitAngle(angle, Units.Radians.of(-Math.PI / 4)));
        this.motor.setSoftwareLimitSwitch(Position.DOWN.position, Position.CLIMB.position);
    }

    private Angle getPosition() {
        return this.motor.getPosition();
    }

    public Command getDownCommand() {
        return runOnce(this::down);
    }

    public void down() {
        this.motor.setPosition(Position.DOWN.position);
        this.position = Position.DOWN;
    }

    public Command getUpCommand() {
        return runOnce(this::up);
    }

    public void up() {
        this.motor.setPosition(Position.UP.position);
        this.position = Position.UP;
    }

    public Command getClimbCommand() {
        return runOnce(this::climb);
    }

    public void climb() {
        this.motor.setPosition(Position.CLIMB.position);
        this.position = Position.CLIMB;
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

    private enum Position {
        DOWN(0),
        UP(1.4),
        CLIMB(4.7);

        final Angle position;

        Position(double rad) {
            this.position = Units.Radians.of(rad);
        }
    }
}

