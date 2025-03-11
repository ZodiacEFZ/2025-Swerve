package frc.robot.subsystem;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.SparkMaxMotor;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.util.Rectangle;

/**
 * The arm on the robot.
 */
public class Arm1 extends SubsystemBase {
    private static final Distance POSTERIOR_ARM_LENGTH = Units.Meters.of(0.86);
    private static final Distance FOREARM_LENGTH = Units.Meters.of(0.77);
    private static final Distance INTAKE_LENGTH = Units.Meters.of(0.25);
    /**
     * The position of the arm installed on the robot. The x component is the distance from the
     * front of the robot to the axis of the first stage of the arm, viewing from the left side of
     * the robot. The y component is the height of the first stage of the arm.
     */
    private static final Translation2d ARM_POSITION = new Translation2d(0.2, 0.24);
    /**
     * The position limit of the arm's components. The x component is the distance from the front of
     * the robot to the axis of the first stage of the arm, viewing from the left side of the robot.
     * The y component is the height of the first stage of the arm.
     */
    // There is no top limit for the arm so the y component is set to 10 meters.
    private static final Rectangle POSITION_LIMIT = new Rectangle(new Translation2d(-0.3, 0),
                                                                  new Translation2d(1, 10));

    private final TalonSRXMotor shoulderLeader = new TalonSRXMotor(21);
    private final TalonSRXMotor shoulderFollower = new TalonSRXMotor(22);
    private final TalonSRXMotor elbow = new TalonSRXMotor(23);
    private final SparkMaxMotor wrist = new SparkMaxMotor(24);
    private Stage2ControlMode stage2ControlMode = Stage2ControlMode.CLOSEST;
    private Pose targetPose;

    /**
     * Constructs an Arm.
     */
    public Arm1() {
        this.shoulderLeader.factoryDefault();
        this.shoulderFollower.factoryDefault();
        this.elbow.factoryDefault();
        this.wrist.factoryDefault();

        // TODO

        this.wrist.setPID(0.1, 0, 0);
        var config = new SparkMaxConfig();
        config.closedLoop.maxMotion.maxAcceleration(0.1).maxVelocity(0.2);
        this.wrist.applyConfiguration(config);
        this.wrist.setSensorToMechanismRatio(100); // 30 * 60 / 18

        this.targetPose = new Pose(this.getStage1Angle(), this.getStage2Angle(),
                                   this.getIntakeRotation());
    }

    private Angle getStage1Angle() {
        return this.shoulderLeader.getPosition();
    }

    private Angle getStage2Angle() {
        return this.elbow.getPosition();
    }

    private Rotation2d getIntakeRotation() {
        return new Rotation2d(this.wrist.getPosition());
    }

    /**
     * Calculates the angles of the stages of the arm with the given position. The first element of
     * the pair is the angle of the first stage. The second element of the pair is the angle of the
     * second stage.
     *
     * @param position  The position of the end of the arm.
     * @param rightHand Whether the arm is in the right hand system.
     *
     * @return A Pair<Angle, Angle>, representing the angles of the stages of the arm.
     */
    private static Pair<Angle, Angle> calculateStageAngles(Translation2d position,
                                                           boolean rightHand) {
        var endPosition = position.minus(ARM_POSITION);
        double l0 = endPosition.getNorm();
        double l1 = POSTERIOR_ARM_LENGTH.in(Units.Meters);
        double l2 = FOREARM_LENGTH.in(Units.Meters);

        double cosTheta2 = (l0 * l0 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        double sinTheta2 = Math.sqrt(1 - cosTheta2 * cosTheta2);
        double theta2 =
                rightHand ? Math.atan2(sinTheta2, cosTheta2) : Math.atan2(-sinTheta2, cosTheta2);
        double theta1 = Math.atan2(endPosition.getY(), endPosition.getX()) -
                        Math.atan2(l2 * sinTheta2, l1 + l2 * cosTheta2);
        return new Pair<>(Units.Radians.of(theta1), Units.Radians.of(theta1 + theta2));
    }

    private static Translation2d calculatePosition(Angle stage1Angle, Angle stage2Angle) {
        double l1 = POSTERIOR_ARM_LENGTH.in(Units.Meters);
        double l2 = FOREARM_LENGTH.in(Units.Meters);
        double theta1 = stage1Angle.in(Units.Radians);
        double theta2 = stage2Angle.in(Units.Radians);
        double x = l1 * Math.cos(theta1) + l2 * Math.cos(theta2);
        double y = l1 * Math.sin(theta1) + l2 * Math.sin(theta2);
        return new Translation2d(x, y);
    }

    private Angle getStage2AngleRelativeToStage1() {
        return this.getStage2Angle().minus(this.getStage1Angle());
    }

    public Command getMoveToClimbCommand() {
        return runOnce(this::moveToClimb);
    }

    private void moveToClimb() {
        this.moveTo(Position.CLIMB.pose);
    }

    public void moveTo(Pose pose) {
        if (isWithinPositionLimit(pose.getTranslation())) {
            this.targetPose = pose;
        }
    }

    private static boolean isWithinPositionLimit(Translation2d position) {
        return POSITION_LIMIT.contains(position);
    }

    public void setStage2ControlMode(Stage2ControlMode stage2ControlMode) {
        this.stage2ControlMode = stage2ControlMode;
    }

    @Override
    public void periodic() {
        // TODO: Implement the control logic to move the arm to the target position.
        this.shoulderLeader.MotionMagic(this.targetPose.getStage1Angle());
        {
            var current = this.getStage2Angle();
            var delta = this.targetPose.getStage2Angle().minus(current);
            var closestDelta = new Rotation2d(Math.cos(delta.in(Units.Radians)),
                                              Math.sin(delta.in(Units.Radians))).getMeasure();
            switch (this.stage2ControlMode) {
                case CLOSEST:
                    this.elbow.MotionMagic(current.plus(closestDelta));
                    break;
                case CLOCKWISE:
                    this.elbow.MotionMagic(closestDelta.in(Units.Radians) > 0 ? closestDelta.minus(
                            Units.Radians.of(Math.PI * 2)) : closestDelta);
                    break;
                case COUNTERCLOCKWISE:
                    this.elbow.MotionMagic(closestDelta.in(Units.Radians) < 0 ? closestDelta.plus(
                            Units.Radians.of(Math.PI * 2)) : closestDelta);
                    break;
            }
        }
        this.wrist.MAXMotionPosition(this.targetPose.getRotation().getMeasure());
    }

    private Translation2d getPosition() {
        return ARM_POSITION.plus(this.getStage2Position());
    }

    private Translation2d getStage2Position() {
        return this.getStage1Position()
                   .plus(new Translation2d(
                           FOREARM_LENGTH.times(Math.cos(this.getStage2Angle().in(Units.Radians))),
                           FOREARM_LENGTH.times(
                                   Math.sin(this.getStage2Angle().in(Units.Radians)))));
    }

    private Translation2d getStage1Position() {
        return new Translation2d(
                POSTERIOR_ARM_LENGTH.times(Math.cos(this.getStage1Angle().in(Units.Radians))),
                POSTERIOR_ARM_LENGTH.times(Math.sin(this.getStage1Angle().in(Units.Radians))));
    }

    enum Position {
        // TODO Add the positions of the arm.
        START(),
        CLIMB(),
        INTAKE(),
        L1(),
        L2(),
        L3(),
        L4();

        private final Pose pose;

        Position() {
            // TODO: delete this
            this.pose = new Pose(new Translation2d(0, 0), Rotation2d.fromDegrees(0), true);
        }

        Position(Pose pose) {
            this.pose = pose;
        }
    }

    public enum Stage2ControlMode {
        CLOSEST,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    public static class Pose {
        private final Pose2d pose;
        private final Angle stage1Angle;
        private final Angle stage2Angle;

        public Pose(Pose2d pose, boolean rightHand) {
            this.pose = pose;
            var angles = calculateStageAngles(pose.getTranslation(), rightHand);
            this.stage1Angle = angles.getFirst();
            this.stage2Angle = angles.getSecond();
        }

        public Pose(Translation2d position, Angle rotation, boolean rightHand) {
            this(position, new Rotation2d(rotation), rightHand);
        }

        public Pose(Translation2d position, Rotation2d rotation, boolean rightHand) {
            this.pose = new Pose2d(position, rotation);
            var angles = calculateStageAngles(pose.getTranslation(), rightHand);
            this.stage1Angle = angles.getFirst();
            this.stage2Angle = angles.getSecond();
        }

        public Pose(Angle stage1Angle, Angle stage2Angle, Angle rotation) {
            this(stage1Angle, stage2Angle, new Rotation2d(rotation));
        }

        public Pose(Angle stage1Angle, Angle stage2Angle, Rotation2d rotation) {
            this.pose = new Pose2d(calculatePosition(stage1Angle, stage2Angle), rotation);
            this.stage1Angle = stage1Angle;
            this.stage2Angle = stage2Angle;
        }

        public Pose2d getPose() {
            return this.pose;
        }

        public Translation2d getTranslation() {
            return this.pose.getTranslation();
        }

        public Rotation2d getRotation() {
            return this.pose.getRotation();
        }

        public Angle getStage1Angle() {
            return this.stage1Angle;
        }

        public Angle getStage2Angle() {
            return this.stage2Angle;
        }

        public Pair<Angle, Angle> getStageAngles() {
            return new Pair<>(this.stage1Angle, this.stage2Angle);
        }
    }
}
