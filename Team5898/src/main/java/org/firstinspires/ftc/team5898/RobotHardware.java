package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static org.firstinspires.ftc.team5898.Constants.Hardware.*;
import static org.firstinspires.ftc.team5898.Constants.Drive.*;

/**
 * <h1>Robot Hardware Abstraction Layer</h1>
 *
 * <p>
 * Provides a centralized interface for initializing and controlling the robot's
 * hardware. This class abstracts FTC SDK hardware access from individual
 * OpModes and provides reusable drivetrain, IMU, and autonomous movement
 * utilities.
 * </p>
 *
 * <p>
 * The class is intended to be instantiated with an {@link OpMode} and initialized
 * by calling {@link #init()} before any hardware-dependent methods are used.
 * </p>
 *
 * <h2>Capabilities</h2>
 * <ul>
 *     <li>Initializes and configures a four-motor mecanum drivetrain</li>
 *     <li>Initializes the REV Hub IMU with the robot's physical orientation</li>
 *     <li>Provides robot-centric and field-centric mecanum driving</li>
 *     <li>Provides IMU heading and heading-error utilities</li>
 *     <li>Provides yaw-reset functionality</li>
 *     <li>Normalizes drivetrain motor power automatically</li>
 *     <li>Applies a configurable joystick deadzone</li>
 *     <li>Provides encoder-based linear and strafing movement</li>
 *     <li>Provides IMU-assisted rotational movement</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * public class MyTeleOp extends OpMode {
 *
 *     private RobotHardware robot;
 *
 *     @Override
 *     public void init() {
 *         robot = new RobotHardware(this);
 *         robot.init();
 *     }
 * }
 * }</pre>
 *
 * @see OpMode
 * @see LinearOpMode
 * @see DcMotorEx
 * @see Servo
 * @see IMU
 */
public class RobotHardware {

    //region Hardware Device Definitions

    // Internal references to the calling OpMode
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Drive motors for the mecanum drive base
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Mechanism motors for game-specific mechanisms
    // public DcMotorEx;

    // Servos for game element manipulators
    // public Servo;

    // Inertial Measurement Unit (IMU) for orientation and field-centric control
    public IMU imu;
   
    //endregion

    //region Drive and Control Constants

    static final double JOYSTICK_DEADZONE = 0.1;
    public double strafeComp = 1.10;

    //endregion

    //region Constructor

    /**
     * Creates a hardware abstraction associated with the specified OpMode.
     *
     * <p>
     * The constructor stores references to the OpMode's {@link HardwareMap}
     * and {@link Telemetry}. Hardware devices are not mapped or initialized
     * until {@link #init()} is called.
     * </p>
     *
     * @param opMode the OpMode that owns and provides access to the robot hardware
     */
    public RobotHardware(@NonNull OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    //endregion

    //region Initialization

    /**
     * Maps and initializes the robot's hardware.
     *
     * <p>
     * This method retrieves the drivetrain motors and IMU from the
     * {@link HardwareMap}, configures motor directions and zero-power behavior,
     * and initializes the IMU using the REV Hub's physical mounting orientation.
     * </p>
     *
     * <p>
     * This method should be called once during the OpMode's initialization phase,
     * before attempting to control the drivetrain or read the IMU.
     * </p>
     *
     * @throws IllegalArgumentException if a configured hardware device cannot
     *                                  be found or has an incompatible type
     */
    public void init() {
        // Map drivetrain motors using the names configured in the Robot Controller app.
        frontLeft  = hardwareMap.get(DcMotorEx.class, MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, MOTOR_FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotorEx.class, MOTOR_BACK_LEFT);
        backRight  = hardwareMap.get(DcMotorEx.class, MOTOR_BACK_RIGHT);

        // Reverse the left-side motors so positive power corresponds to forward motion.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Brake the drivetrain when power is set to zero.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Map and initialize the IMU.
        //
        // These directions must match the physical orientation of the REV Hub.
        // An incorrect orientation will produce incorrect heading measurements
        // and can cause field-centric driving to behave incorrectly.
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();
    }

    //endregion

    //region IMU and Heading Utilities

    /**
     * Resets the IMU's yaw angle to zero.
     *
     * <p>
     * After this method is called, subsequent yaw measurements are relative
     * to the robot's orientation at the time of the reset. This is useful for
     * establishing a new heading reference before field-centric driving or
     * autonomous rotation.
     * </p>
     *
     * <p>
     * Resetting yaw does not physically rotate the robot and does not reset
     * pitch or roll.
     * </p>
     */
    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    /**
     * Returns the robot's current IMU yaw angle in radians.
     *
     * <p>The returned value is normalized to the range {@code [-π, π)}.</p>
     *
     * @return current robot heading in radians
     *
     * @see #getHeadingDeg()
     * @see #resetYaw()
     */
    @CheckResult(suggest = "heading = getHeadingRad()")
    public double getHeadingRad() {
        return AngleUnit.normalizeRadians(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }

    /**
     * Returns the robot's current IMU yaw angle in degrees.
     *
     * <p>The returned value is normalized to the range {@code [-180°, 180°)}.</p>
     *
     * @return current robot heading in degrees
     *
     * @see #getHeadingRad()
     * @see #resetYaw()
     */
    @CheckResult(suggest = "heading = getHeadingDeg()")
    public double getHeadingDeg() {
        return AngleUnit.normalizeDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }

    /**
     * Calculates the shortest signed angular error between two headings.
     *
     * <p>
     * The result is calculated as {@code target - current} and normalized to
     * the range {@code [-180°, 180°)}. A positive result indicates that the
     * target is clockwise from the current heading according to the heading
     * convention used by the IMU.
     * </p>
     *
     * @param current current heading in degrees
     * @param target desired heading in degrees
     *
     * @return shortest signed angular error from {@code current} to {@code target},
     *         in degrees
     */
    @CheckResult(suggest = "error = getHeadingError(current, target)")
    public double getHeadingError(double current, double target) {
        double error = target - current;

        // Normalize the error to the shortest path: [-180°, 180°).
        while (error > 180) {
            error -= 360;
        }

        while (error <= -180) {
            error += 360;
        }

        return error;
    }

    //endregion

    //region Drive Utilities and Input Processing

    /**
     * Sets drivetrain motor power while automatically normalizing the inputs.
     *
     * <p>
     * If every requested power is already within {@code [-1, 1]}, the values
     * are applied unchanged. If any requested value exceeds that range, all
     * values are scaled proportionally so that the largest magnitude becomes
     * exactly {@code 1.0}.
     * </p>
     *
     * <p>
     * This preserves the relative ratios between the four motor powers while
     * preventing any motor from receiving a value outside the valid FTC SDK
     * power range.
     * </p>
     *
     * @param fl desired front-left motor power
     * @param fr desired front-right motor power
     * @param bl desired back-left motor power
     * @param br desired back-right motor power
     */
    public void setDrivePower(double fl, double fr, double bl, double br) {
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    /**
     * Stops all drivetrain motors by setting their power to zero.
     *
     * <p>The motors retain their configured zero-power behavior.</p>
     *
     * @see #setDrivePower(double, double, double, double)
     */
    public void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Sets the same {@link DcMotor.RunMode} on all four drivetrain motors.
     *
     * @param mode run mode to apply to every drivetrain motor
     */
    public void setRunMode(@NonNull DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    /**
     * Applies the configured joystick deadzone and rescales the remaining input.
     *
     * <p>
     * Inputs with an absolute value smaller than {@link #JOYSTICK_DEADZONE}
     * are converted to zero. Larger inputs are shifted and rescaled so that
     * the first nonzero output occurs immediately outside the deadzone and
     * full joystick travel still produces an output of {@code ±1}.
     * </p>
     *
     * @param input raw joystick input, normally in the range {@code [-1, 1]}
     *
     * @return deadzone-adjusted joystick input in the range {@code [-1, 1]}
     */
    @CheckResult(suggest = "input = applyJoystickDeadzone(input)")
    public double applyJoystickDeadzone(double input) {
        if (Math.abs(input) < JOYSTICK_DEADZONE) {
            return 0.0;
        }

        return (input - Math.signum(input) * JOYSTICK_DEADZONE)
                / (1.0 - JOYSTICK_DEADZONE);
    }

    //endregion

    //region Drivetrain Control Methods

    /**
     * Drives the robot using robot-centric mecanum control.
     *
     * <p>
     * The translation inputs are interpreted relative to the robot itself.
     * For example, a positive forward input always commands motion toward the
     * robot's front, regardless of its orientation on the field.
     * </p>
     *
     * @param x  lateral translation input; positive values command rightward motion
     * @param y  longitudinal translation input; positive values command forward motion
     * @param rx rotational input; positive values command clockwise rotation
     *
     * @see #driveFieldCentric(double, double, double)
     */
    public void driveRobotCentric(double x, double y, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        setDrivePower(fl, fr, bl, br);
    }

    /**
     * Drives the robot using field-centric mecanum control.
     *
     * <p>
     * The translation inputs are interpreted relative to the field rather than
     * the robot. The IMU heading is used to rotate the requested translation
     * vector into the robot's coordinate system before applying mecanum
     * drive calculations.
     * </p>
     *
     * <p>
     * The configured {@link #strafeComp} factor is applied to the lateral input
     * before the field-to-robot coordinate transformation.
     * </p>
     *
     * @param x  lateral field-relative translation input; positive values command
     *           rightward motion
     * @param y  longitudinal field-relative translation input; positive values
     *           command forward motion
     * @param rx rotational input; positive values command clockwise rotation
     *
     * @see #driveRobotCentric(double, double, double)
     * @see #getHeadingRad()
     */
    public void driveFieldCentric(double x, double y, double rx) {
        x *= strafeComp;

        double heading = getHeadingRad();

        // Rotate the field-relative translation vector into robot coordinates.
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        driveRobotCentric(rotX, rotY, rx);
    }

    //endregion

    //region Autonomous Drivetrain Movement

    /**
     * Returns the owning OpMode as a {@link LinearOpMode}, if applicable.
     *
     * <p>
     * The blocking autonomous movement methods in this class require a
     * {@code LinearOpMode} because they depend on its lifecycle methods,
     * particularly {@link LinearOpMode#opModeIsActive()} and
     * {@link LinearOpMode#idle()}.
     * </p>
     *
     * @return the owning OpMode as a {@link LinearOpMode}, or {@code null} if
     *         the owning OpMode is not a LinearOpMode
     */
    private LinearOpMode getLinearOpMode() {
        if (!(opMode instanceof LinearOpMode)) {
            return null;
        }

        return (LinearOpMode) opMode;
    }

    /**
     * Moves the robot forward or backward by a specified distance using
     * drivetrain encoders.
     *
     * <p>
     * The requested distance is converted from inches to encoder counts using
     * {@link Constants.Drive#COUNTS_PER_INCH}. All four motors are commanded
     * to move by the same number of encoder counts.
     * </p>
     *
     * <p>
     * This method is blocking and returns when the OpMode becomes inactive or
     * the drivetrain reaches its encoder targets.
     * </p>
     *
     * <p>
     * If this object was created with an {@link OpMode} that is not a
     * {@link LinearOpMode}, the method stops the drivetrain and returns
     * immediately.
     * </p>
     *
     * @param inches distance to travel in inches; positive values move forward,
     *               negative values move backward
     * @param speed motor power magnitude, normally in the range {@code [0, 1]}
     *
     * @see #strafeToPosition(double, double)
     * @see #setRunMode(DcMotor.RunMode)
     */
    @WorkerThread
    public void moveToPosition(double inches, double speed) {
        LinearOpMode linearOpMode = getLinearOpMode();

        if (linearOpMode == null) {
            stopDrive();
            return;
        }

        // Convert the requested distance to encoder counts.
        int moveCounts = (int) Math.round(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + moveCounts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + moveCounts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + moveCounts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + moveCounts);

        // Command all drivetrain motors to move toward their targets.
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Math.abs(speed);
        setDrivePower(power, power, power, power);

        // Continue while the OpMode is active and all motors are still moving.
        while (linearOpMode.opModeIsActive()
                && frontLeft.isBusy()
                && frontRight.isBusy()
                && backLeft.isBusy()
                && backRight.isBusy()) {

            telemetry.addData("Drive", "Moving...");
            telemetry.update();

            linearOpMode.idle();
        }

        stopDrive();
    }

    /**
     * Rotates the robot by the specified number of degrees using the IMU for
     * heading feedback.
     *
     * <p>
     * The requested rotation is relative to the robot's heading at the time
     * this method is called. Positive and negative values determine the desired
     * direction of rotation through the supplied {@code speedDirection}.
     * </p>
     *
     * <p>
     * Rotation is performed in two stages:
     * </p>
     * <ol>
     *     <li>A coarse rotation continues until the heading error is below
     *         approximately 10 degrees.</li>
     *     <li>A slower fine rotation continues until the heading error is below
     *         2 degrees.</li>
     * </ol>
     *
     * <p>
     * This method is blocking and returns when the target heading is reached
     * or the OpMode becomes inactive.
     * </p>
     *
     * @param degrees relative rotation in degrees; positive and negative values
     *                represent opposite rotation directions
     * @param speedDirection signed motor power used for the coarse rotation;
     *                       its magnitude should normally be in {@code [0, 1]}
     *
     * @see #getHeadingDeg()
     * @see #getHeadingError(double, double)
     * @see #turnWithEncoder(double)
     */
    @WorkerThread
    public void turnWithGyro(double degrees, double speedDirection) {
        LinearOpMode linearOpMode = getLinearOpMode();

        if (linearOpMode == null) {
            stopDrive();
            return;
        }

        double heading = getHeadingDeg();

        // Calculate the target heading and normalize it to [-180°, 180°).
        double targetHeading = AngleUnit.normalizeDegrees(heading + degrees);

        // Coarse rotation.
        while (linearOpMode.opModeIsActive()) {
            heading = getHeadingDeg();
            double error = getHeadingError(heading, targetHeading);

            if (Math.abs(error) < 10) {
                break;
            }

            turnWithEncoder(speedDirection);
        }

        // Fine rotation at one-third of the coarse-turn power.
        while (linearOpMode.opModeIsActive()) {
            heading = getHeadingDeg();
            double error = getHeadingError(heading, targetHeading);

            if (Math.abs(error) < 2) {
                break;
            }

            turnWithEncoder(speedDirection / 3);
        }

        stopDrive();
    }

    /**
     * Applies open-loop rotational power to the drivetrain.
     *
     * <p>
     * Opposite power is applied to the left and right sides, causing the robot
     * to rotate in place. This method does not use encoder position or IMU
     * feedback to determine when the rotation is complete.
     * </p>
     *
     * @param input signed rotational motor power; the sign determines the
     *             direction of rotation and the magnitude determines speed
     *
     * @see #turnWithGyro(double, double)
     */
    public void turnWithEncoder(double input) {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(input);
        frontRight.setPower(-input);
        backLeft.setPower(input);
        backRight.setPower(-input);
    }

    /**
     * Strafes the robot by a specified distance using drivetrain encoders.
     *
     * <p>
     * The requested distance is converted to encoder counts using
     * {@link Constants.Drive#COUNTS_PER_INCH} and {@link #strafeComp}.
     * The front-left and back-right motors move in the opposite encoder
     * direction from the front-right and back-left motors to produce lateral
     * movement.
     * </p>
     *
     * <p>
     * Positive distances command strafing in one direction and negative
     * distances command strafing in the opposite direction. The exact physical
     * direction depends on the drivetrain motor configuration.
     * </p>
     *
     * <p>
     * This method is blocking and returns when the OpMode becomes inactive or
     * the drivetrain reaches its encoder targets.
     * </p>
     *
     * @param inches distance to strafe in inches
     * @param speed motor power magnitude, normally in the range {@code [0, 1]}
     *
     * @see #moveToPosition(double, double)
     * @see #strafeComp
     */
    @WorkerThread
    public void strafeToPosition(double inches, double speed) {
        LinearOpMode linearOpMode = getLinearOpMode();

        if (linearOpMode == null) {
            stopDrive();
            return;
        }

        // Convert the requested lateral distance to encoder counts.
        double moveCounts = inches * COUNTS_PER_INCH * strafeComp;
        int counts = (int) Math.round(moveCounts);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + counts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - counts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - counts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + counts);

        // Command all drivetrain motors to move toward their targets.
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Math.abs(speed);
        setDrivePower(power, power, power, power);

        // Continue while the OpMode is active and all motors are still moving.
        while (linearOpMode.opModeIsActive()
                && frontLeft.isBusy()
                && frontRight.isBusy()
                && backLeft.isBusy()
                && backRight.isBusy()) {

            telemetry.addData("Drive", "Strafing...");
            telemetry.update();

            linearOpMode.idle();
        }

        stopDrive();
    }

    //endregion
}
