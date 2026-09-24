package org.firstinspires.ftc.team5898

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.team5898.Constants.CannonConstants
import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing
import kotlin.concurrent.timer
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@Disabled
@TeleOp(name = "Flywheel Test", group = "TeleOP")
class Graph_PID_Flywheel : OpMode() {
    private val timer = ElapsedTime()
    // Late initialization for hardware variables
    private lateinit var limelight: Limelight3A
    private lateinit var frontLeft: DcMotor
    private lateinit var frontRight: DcMotor
    private lateinit var backLeft: DcMotor
    private lateinit var backRight: DcMotor
    private lateinit var topLauncher: DcMotorEx
    private lateinit var bottomLauncher: DcMotorEx
    private lateinit var backLeftServo: CRServo
    private lateinit var backRightServo: CRServo
    private lateinit var frontIntake: DcMotor
    private lateinit var imu: IMU
    private var targetVelocity = CannonConstants.BACK_LAUNCH_VELOCITY
    private var topCurrentVelocity: Double = 0.0
    private var bottomCurrentVelocity: Double = 0.0
    private var panelsTelemetry = PanelsTelemetry.telemetry
    private var visualServoing: VisualServoing? = null

    // Constants variables
    private var intakePower: Double = 0.0
    private var LAUNCH_VELOCITY: Double = 0.0

    // PID Gains (Using Int/Double to match the types in the Java code)
    private var kP: Double = 0.0
    private var kI: Double = 0.0
    private var kD: Double = 0.0
    private var kF: Double = 0.0

    override fun init() {
        // Constants Init
        // Cannon Constants
        val t = timer.seconds()
        intakePower = CannonConstants.IntakePower


        panelsTelemetry.addData("timer", t)
        panelsTelemetry.addData("Launch Velocity", LAUNCH_VELOCITY)
        panelsTelemetry.addData("topLauncher Velocity", topCurrentVelocity)
        panelsTelemetry.addData("bottomLauncher Velocity", bottomCurrentVelocity)
        panelsTelemetry.update(telemetry)

        kP = CannonConstants.kP
        kI = CannonConstants.kI
        kD = CannonConstants.kD
        kF = CannonConstants.kF



        // Limelight Init
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)
        telemetry.setMsTransmissionInterval(11)
        limelight.setPollRateHz(90)
        limelight.start()

        // Motors Init
        frontLeft = hardwareMap.get(DcMotor::class.java, "FL")
        frontRight = hardwareMap.get(DcMotor::class.java, "FR")
        backLeft = hardwareMap.get(DcMotor::class.java, "BL")
        backRight = hardwareMap.get(DcMotor::class.java, "BR")
        topLauncher = hardwareMap.get(DcMotorEx::class.java, "TLaunch")
        bottomLauncher = hardwareMap.get(DcMotorEx::class.java, "BLaunch")
        frontIntake = hardwareMap.get(DcMotor::class.java, "Intake")

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        frontRight.direction = DcMotorSimple.Direction.FORWARD
        backRight.direction = DcMotorSimple.Direction.FORWARD

        topLauncher.direction = DcMotorSimple.Direction.FORWARD
        bottomLauncher.direction = DcMotorSimple.Direction.FORWARD

        // Set motors to use encoders for velocity control. This is a crucial step.
        topLauncher.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        bottomLauncher.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        topLauncher.mode = DcMotor.RunMode.RUN_USING_ENCODER
        bottomLauncher.mode = DcMotor.RunMode.RUN_USING_ENCODER

        // Improved PIDF coefficients for more stable velocity control
        // P: Proportional gain - increased for faster response
        // I: Integral gain - added to eliminate steady-state error
        // D: Derivative gain - added to reduce overshoot and oscillation
        // F: Feed-forward gain - tuned for velocity control
        // Tune these values based on your motor's behavior:
        // - If oscillating: decrease P, increase D
        // - If slow to reach target: increase P, increase F
        // - If steady-state error: increase I (start small, like 0.1-0.5)
        topLauncher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(kP.toDouble(), kI, kD, kF.toDouble())
        )
        bottomLauncher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(kP.toDouble(), kI, kD, kF.toDouble())
        )

        // Set zero power behavior to FLOAT for launchers (reduces resistance)
        topLauncher.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        bottomLauncher.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        backLeftServo = hardwareMap.get(CRServo::class.java, "LServo")
        backRightServo = hardwareMap.get(CRServo::class.java, "RServo")

        frontIntake.direction = DcMotorSimple.Direction.FORWARD
        backLeftServo.direction = DcMotorSimple.Direction.REVERSE
        backRightServo.direction = DcMotorSimple.Direction.FORWARD

        // IMU Init for Field-Centric
        imu = hardwareMap.get(IMU::class.java, "imu")
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        )
        imu.initialize(parameters)

        // Visual Servoing Init
        visualServoing = VisualServoing(limelight, frontLeft, frontRight, backRight, backLeft, telemetry)

        LAUNCH_VELOCITY = CannonConstants.BACK_LAUNCH_VELOCITY

        topCurrentVelocity = abs(topLauncher.getVelocity())
        bottomCurrentVelocity = abs(bottomLauncher.getVelocity())
        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        LAUNCH_VELOCITY = CannonConstants.BACK_LAUNCH_VELOCITY
        val t = timer.seconds()
        topCurrentVelocity = abs(topLauncher.getVelocity())
        bottomCurrentVelocity = abs(bottomLauncher.getVelocity())
        panelsTelemetry.addData("timer", t)
        panelsTelemetry.addData("Launch Velocity", LAUNCH_VELOCITY)
        panelsTelemetry.addData("topLauncher Velocity", topCurrentVelocity)
        panelsTelemetry.addData("bottomLauncher Velocity", bottomCurrentVelocity)
        panelsTelemetry.update(telemetry)

        // Update PID constants (if Constants are dynamic)
        kP = CannonConstants.kP
        kI = CannonConstants.kI
        kD = CannonConstants.kD
        kF = CannonConstants.kF

        topLauncher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(kP.toDouble(), kI, kD, kF.toDouble())
        )
        bottomLauncher.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(kP.toDouble(), kI, kD, kF.toDouble())
        )

        // VisualServoing: gamepad1.a
        // Check if VisualServoing
        val isVisualServoing = gamepad1.a
        if (isVisualServoing) {
            visualServoing?.visualServo()
        }

        // Cannon Control - Now using setVelocity for closed-loop control
        if (gamepad2.left_stick_y > 0.3) {
            topLauncher.velocity = -LAUNCH_VELOCITY
            bottomLauncher.velocity = LAUNCH_VELOCITY
        } else if (gamepad2.left_stick_y < -0.3) {
            topLauncher.velocity = LAUNCH_VELOCITY
            bottomLauncher.velocity = -LAUNCH_VELOCITY
        } else {
            topLauncher.velocity = 0.0
            bottomLauncher.velocity = 0.0
        }

        // Intake and Transfer Control
        if (gamepad2.right_stick_y > 0.3) {
            backLeftServo.power = -intakePower
            backRightServo.power = -intakePower
            frontIntake.power = -intakePower
        } else if (gamepad2.right_stick_y < -0.3) {
            backLeftServo.power = intakePower
            backRightServo.power = intakePower
            frontIntake.power = 0.7
        } else {
            backLeftServo.power = 0.0
            backRightServo.power = 0.0
            frontIntake.power = 0.0
        }

        // DPad Control for fine adjustment
        if (gamepad2.dpad_down) {
            backLeftServo.power = intakePower - 0.3
            backRightServo.power = intakePower - 0.3
        } else if (gamepad2.dpad_up) {
            backLeftServo.power = -intakePower + 0.3
            backRightServo.power = -intakePower + 0.3
        }

        // ===========================================

        // Field-Centric Drive Code (skip if Visual Servoing is active)
        if (!isVisualServoing) {
            val y = -gamepad1.left_stick_y.toDouble() // Forward/Backward (reversed)
            val x = gamepad1.left_stick_x * 1.1 // Strafe Left/Right (counteract imperfect strafing)
            val rx = gamepad1.right_stick_x.toDouble() // Rotation

            // Reset IMU yaw with guide button
            if (gamepad1.guide) {
                imu.resetYaw()
            }

            // Get robot heading from IMU
            val botHeading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            // Rotate movement direction based on robot's heading
            val rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            // Calculate motor powers
            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            val frontLeftPower = (rotY + rotX + rx) / denominator
            val backLeftPower = (rotY - rotX + rx) / denominator
            val frontRightPower = (rotY - rotX - rx) / denominator
            val backRightPower = (rotY + rotX - rx) / denominator

            if (!gamepad1.left_bumper) {
                // Set motor powers (adjust multiplier as needed)
                frontLeft.power = frontLeftPower
                backLeft.power = backLeftPower
                frontRight.power = frontRightPower
                backRight.power = backRightPower
            } else {
                // Set motor powers with 0.5 multiplier (Slow mode)
                frontLeft.power = frontLeftPower * 0.5
                backLeft.power = backLeftPower * 0.5
                frontRight.power = frontRightPower * 0.5
                backRight.power = backRightPower * 0.5
            }
        }
    }

    override fun stop() {
        limelight.stop()
    }
}