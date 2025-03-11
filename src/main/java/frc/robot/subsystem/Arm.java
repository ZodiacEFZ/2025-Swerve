package frc.robot.subsystem;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.SparkMaxMotor;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.util.Maths;
import frc.libzodiac.util.Rectangle;
import frc.libzodiac.util.SimpleSendable;

import java.util.Optional;

/**
 * The arm on the robot.
 */
public final class Arm extends SubsystemBase implements SimpleSendable {
    private static final Distance POSTERIOR_ARM_LENGTH = Units.Meters.of(0.86);
    private static final Distance FOREARM_LENGTH = Units.Meters.of(0.77);
    private static final Distance INTAKE_LENGTH = Units.Meters.of(0.25);
    /**
     * Displacement from robot center to base of the arm, in the <a
     * href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">
     * <i>WPILib coordinate system</i></a>.
     */
    // TODO: fill in correct x and y
    private static final Translation3d INSTALL_POSITION = new Translation3d(0, 0, 0.24);
    private final TalonSRXMotor shoulderLeader = new TalonSRXMotor(21);
    private final TalonSRXMotor shoulderFollower = new TalonSRXMotor(22);
    private final TalonSRXMotor elbow = new TalonSRXMotor(23);
    private final SparkMaxMotor wrist = new SparkMaxMotor(24);

    /**
     * The position limit of the arm's components on the <a
     * href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system">XZ
     * plane</a> with respect to robot center.
     */
    // There is no top limit for the arm so the y component is set to 10 meters.
    private final Rectangle positionLimit = new Rectangle(new Translation2d(-0.3, 0),
                                                          new Translation2d(1, 10));

    /**
     * The current desired position of the arm.
     */
    @SendableProperty
    private Position target = Position.IDLE;

    /**
     * Constructs an Arm.
     */
    public Arm() {
        this.shoulderLeader.factoryDefault();
        this.shoulderFollower.factoryDefault();
        this.elbow.factoryDefault();
        this.wrist.factoryDefault();

        // TODO: tune these arguments
        this.shoulderLeader.setMotionMagicConfig(0, 0, 0, 0, Units.RadiansPerSecond.of(0),
                                                 Units.RadiansPerSecondPerSecond.of(0), 0);
        this.elbow.setMotionMagicConfig(0, 0, 0, 0, Units.RadiansPerSecond.of(0),
                                        Units.RadiansPerSecondPerSecond.of(0), 0);

        this.wrist.setPID(0.1, 0, 0);
        var config = new SparkMaxConfig();
        config.closedLoop.maxMotion.maxAcceleration(0.1).maxVelocity(0.2);
        this.wrist.applyConfiguration(config);
        this.wrist.setSensorToMechanismRatio(100); // 30 * 60 / 18
    }

    /**
     * Projection of arm installation position on the XZ plane (the plane in which the arm moves).
     *
     * @return {@link Translation2d}
     */
    private static Translation2d installPositionXZ() {
        return new Translation2d(INSTALL_POSITION.getX(), INSTALL_POSITION.getZ());
    }

    /**
     * According to the current velocity of arm, calculate ETA at the current target.
     *
     * @return the estimated time, empty if the arm is not currently moving towards the target
     */
    @SendableGetter (name = "ETA")
    public Optional<Time> eta() {
        final var curr = this.getPosition();
        if (curr.isNear(this.target, Units.Degree.of(5))) {
            return Optional.of(Units.Second.of(0));
        }
        final var shoulder = this.target.shoulder.minus(curr.shoulder).in(Units.Radian) /
                             this.shoulderLeader.getVelocity().in(Units.RadiansPerSecond);
        final var elbow = this.target.elbow.minus(curr.shoulder).in(Units.Radian) /
                          this.elbow.getVelocity().in(Units.RadiansPerSecond);
        final var eta = Math.max(shoulder, elbow);
        if (!Double.isFinite(eta) || eta < 0) {
            return Optional.empty();
        }
        return Optional.of(Units.Second.of(eta));
    }

    /**
     * Retrieve current mechanism position.
     *
     * @return the {@link Position}
     */
    @SendableGetter (name = "Current Position")
    public Position getPosition() {
        return new Position(this.shoulderLeader.getPosition(), this.elbow.getPosition(),
                            this.wrist.getPosition());
    }

    @Override
    public void periodic() {
        this.shoulderLeader.MotionMagic(this.target.shoulder);
        this.elbow.MotionMagic(this.target.elbow);
        this.wrist.MAXMotionPosition(this.target.wrist);
    }

    public void moveTo(Position target) {
        if (!this.isWithinPositionLimit(target.wristPosition())) {
            throw new IllegalArgumentException("the target position is not attainable");
        }
        this.target = target;
    }

    private boolean isWithinPositionLimit(Translation2d position) {
        return this.positionLimit.contains(position);
    }

    public Command follow(Trajectory trajectory) {
        return new FollowTrajectory(this, trajectory);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        this.simpleSendableInit(builder);
    }

    /**
     * Represents a certain state of arm.
     *
     * @param shoulder the angle between posterior arm and robot front
     * @param elbow    the angle between forearm and posterior arm
     * @param wrist    the angle of wrist rotation
     */
    public record Position(
            @SendableProperty (access = SendableProperty.AccessType.Out) Angle shoulder,
            @SendableProperty (access = SendableProperty.AccessType.Out) Angle elbow,
            @SendableProperty (access = SendableProperty.AccessType.Out) Angle wrist)
            implements SimpleSendable {
        // TODO: Fill in these values
        public static final Position IDLE = new Position(Units.Radian.of(0), Units.Radian.of(0),
                                                         Units.Radian.of(0));
        public static final Position INTAKE = IDLE;
        public static final Position CLIMB = IDLE;
        public static final Position L1 = IDLE;
        public static final Position L2 = IDLE;
        public static final Position L3 = IDLE;
        public static final Position L4 = IDLE;

        /**
         * Calculate the arm position that implements the specified total displacement.
         *
         * @param displacement desired displacement with respect to the base of arm in meters
         *
         * @return both two solutions, the first one is always with smaller angle between posterior
         * arm and robot front
         */
        public static Pair<Position, Position> resolve(Translation2d displacement) {
            final var l0 = displacement.getNorm();
            final var l1 = POSTERIOR_ARM_LENGTH.in(Units.Meter);
            final var l2 = FOREARM_LENGTH.in(Units.Meter);

            final var t1 = Maths.resolveAngle(l2, l0, l1);
            final var t2 = Maths.resolveAngle(l0, l1, l2);

            final var arg = displacement.getAngle().getMeasure().in(Units.Radian);

            final var shoulder1 = arg - t1;
            final var shoulder2 = arg + t1;
            final var elbow1 = Math.PI - t2;
            final var elbow2 = -elbow1;

            final var sol1 = new Position(Units.Radian.of(shoulder1), Units.Radian.of(elbow1),
                                          Units.Radian.of(0));
            final var sol2 = new Position(Units.Radian.of(shoulder2), Units.Radian.of(elbow2),
                                          Units.Radian.of(0));

            return new Pair<>(sol1, sol2);
        }

        public boolean isNear(Position rhs) {
            // TODO: this threshold shall be tuned
            return this.isNear(rhs, Units.Degree.of(15));
        }

        public boolean isNear(Position rhs, Angle threshold) {
            return shoulder.isNear(rhs.shoulder, threshold) && elbow.isNear(rhs.elbow, threshold);
        }

        public Translation2d wristPosition() {
            return new Translation2d(FOREARM_LENGTH.in(Units.Meter),
                                     new Rotation2d(this.shoulder.plus(this.elbow))).plus(
                    this.elbowPosition());
        }

        public Translation2d elbowPosition() {
            return new Translation2d(POSTERIOR_ARM_LENGTH.in(Units.Meter),
                                     new Rotation2d(this.shoulder)).plus(installPositionXZ());
        }
    }

    public record Trajectory(Position[] waypoints) {

        public Trajectory followedBy(Trajectory next) {
            var buf = new Position[this.waypoints.length + next.waypoints.length];
            System.arraycopy(this.waypoints, 0, buf, 0, this.waypoints.length);
            System.arraycopy(next.waypoints, 0, buf, this.waypoints.length, next.waypoints.length);
            return new Trajectory(buf);
        }
    }

    private static final class FollowTrajectory extends Command {

        private static final Time SWITCH_THRESHOLD = Units.Millisecond.of(200);

        private final Arm arm;
        private final Trajectory trajectory;
        private int destination = 0;

        public FollowTrajectory(Arm arm, Trajectory trajectory) {
            this.arm = arm;
            this.trajectory = trajectory;
        }

        @Override
        public void execute() {
            final var curr = this.arm.getPosition();
            if (curr.isNear(this.trajectory.waypoints[this.destination])) {
                this.destination++;
            } else {
                // switch to the next waypoint as well if we are to arrive shortly
                final var eta = this.arm.eta();
                if (eta.isPresent()) {
                    if (eta.get().lt(SWITCH_THRESHOLD)) {
                        this.destination++;
                    }
                }
            }
            if (this.isFinished()) {
                return;
            }
            this.arm.moveTo(this.trajectory.waypoints[this.destination]);
        }

        @Override
        public boolean isFinished() {
            return this.destination >= this.trajectory.waypoints.length;
        }
    }
}
