package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;

@TeleOp(name = "Beta TeleOP")
public class Beta_TeleOP_Refactor extends OpMode {
    ElapsedTime timer = new ElapsedTime();
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotorEx topLauncher, bottomLauncher;
    VisualServoing visualServoing;
    CRServo backLeftServo, backRightServo;
    Servo stopperServo;
    DcMotor frontIntake;
    IMU imu;

    double intakePower;
    double FRONT_LAUNCH_VELOCITY, BACK_LAUNCH_VELOCITY, LAUNCH_VELOCITY;
    double stopperClosePosition = CannonConstants.stopperClosePosition;
    double stopperOpenPosition = CannonConstants.stopperOpenPosition;
    double kP, kI, kD, kF;
    enum States {IDLE, INTAKE, LAUNCH, SPOOL_UP, VISUAL_SERVOING}
    boolean shootingFromBack = false;
    private States robotState = States.IDLE;

    // Button state variables
    boolean prevA1 = false, prevB2 = false, prevA2 = false;

    @Override
    public void init() {
        // Constants Init
        intakePower = CannonConstants.IntakePower;
        kP = CannonConstants.kP;
        kI = CannonConstants.kI;
        kD = CannonConstants.kD;
        kF = CannonConstants.kF;
        FRONT_LAUNCH_VELOCITY = -(CannonConstants.FRONT_LAUNCH_VELOCITY);
        BACK_LAUNCH_VELOCITY = -(CannonConstants.BACK_LAUNCH_VELOCITY);
        stopperClosePosition = CannonConstants.stopperClosePosition;
        stopperOpenPosition = CannonConstants.stopperOpenPosition;

        // Limelight Init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(90);
        limelight.start();

        // Motors Init
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        topLauncher = hardwareMap.get(DcMotorEx.class, "TLaunch");
        bottomLauncher = hardwareMap.get(DcMotorEx.class, "BLaunch");
        frontIntake = hardwareMap.get(DcMotor.class, "Intake");

        // Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD); // Added missing direction

        topLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Mode setup
        topLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        bottomLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        topLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backLeftServo = hardwareMap.get(CRServo.class, "LServo");
        backRightServo = hardwareMap.get(CRServo.class, "RServo");
        stopperServo = hardwareMap.get(Servo.class, "STPR");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.FORWARD);

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU Init
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Visual Servoing Init
        visualServoing = new VisualServoing(limelight, frontLeft, frontRight, backRight, backLeft, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


    }

    @Override
    public void loop() {
        // 1. Update button states
        boolean a1Pressed = gamepad1.a;
        boolean b2Pressed = gamepad2.b;
        boolean a2Pressed = gamepad2.a;
        boolean rightTrigger2 = gamepad2.right_trigger > 0.3; // Threshold for trigger

        boolean a1JustPressed = a1Pressed && !prevA1;
        boolean b2JustPressed = b2Pressed && !prevB2;
        boolean a2JustPressed = a2Pressed && !prevA2;

        LLResult Results = limelight.getLatestResult();
        if (Results != null && Results.isValid() && Results.getTa() < 1.5 && !shootingFromBack) {
            shootingFromBack = true;
        } else if (Results != null && Results.isValid() && Results.getTa() >= 1.5 && shootingFromBack) {
            shootingFromBack = false;
        } else if (Results == null || !Results.isValid()) {
            shootingFromBack = false;
        }

        if (shootingFromBack) {
            LAUNCH_VELOCITY = BACK_LAUNCH_VELOCITY;
        } else {
            LAUNCH_VELOCITY = FRONT_LAUNCH_VELOCITY;
        }


        // 2. State machine logic
        switch (robotState) {
            case IDLE:
                FieldCentricDrive(false);

                // Stop all motors
                backLeftServo.setPower(0);
                backRightServo.setPower(0);
                frontIntake.setPower(0);
                topLauncher.setPower(0);
                bottomLauncher.setPower(0);
                stopperServo.setPosition(stopperClosePosition); // Close the stopper

                // State transitions
                if (b2JustPressed) {
                    robotState = States.INTAKE;
                } else if (a1JustPressed) {
                    robotState = States.VISUAL_SERVOING;
                } else if (gamepad2.right_trigger > 0.3) {
                    robotState = States.SPOOL_UP;
                }
                break;

            case INTAKE:
                FieldCentricDrive(false);

                // Intake action
                backLeftServo.setPower(intakePower);
                backRightServo.setPower(intakePower);
                frontIntake.setPower(0.7);
                stopperServo.setPosition(stopperClosePosition); // Ensure stopper is closed

                // State transitions
                if (b2JustPressed) {
                    robotState = States.IDLE; // Press B again to stop
                } else if (rightTrigger2) {
                    robotState = States.SPOOL_UP; // Press Trigger to start spooling up
                }
                break;

            case SPOOL_UP:
                FieldCentricDrive(false);

                // Launcher wheel acceleration
                topLauncher.setVelocity(LAUNCH_VELOCITY);
                bottomLauncher.setVelocity(LAUNCH_VELOCITY);

                // Optional: keep intake running or stop. Here we stop intake while waiting to launch to prevent jamming
                backLeftServo.setPower(0);
                backRightServo.setPower(0);
                frontIntake.setPower(0);
                stopperServo.setPosition(stopperOpenPosition); // Stopper open

                // State transitions
                if (!rightTrigger2) {
                    robotState = States.IDLE; // Release Trigger to return to IDLE
                } else if (a2JustPressed) {
                    robotState = States.LAUNCH; // Press A to launch
                }
                break;

            case LAUNCH:
                FieldCentricDrive(false);

                // Maintain launcher wheel velocity
                topLauncher.setVelocity(LAUNCH_VELOCITY);
                bottomLauncher.setVelocity(LAUNCH_VELOCITY);

                // Open stopper and feed the ring
                stopperServo.setPosition(stopperOpenPosition); // Move stopper away
                backLeftServo.setPower(1);   // Feed servos/motors
                backRightServo.setPower(1);
                frontIntake.setPower(.7);

                // State transitions
                // If A is released, return to SPOOL_UP (keep spinning, prep for next shot)
                if (!a2Pressed) {
                    robotState = States.SPOOL_UP;
                }
                // If Trigger is released, return to IDLE
                if (!rightTrigger2) {
                    robotState = States.IDLE;
                }
                break;

            case VISUAL_SERVOING:
                FieldCentricDrive(true);
                visualServoing.visualServo();

                // State transitions
                if (a1JustPressed) {
                    robotState = States.IDLE;
                } else if (visualServoing.isAligned()) {
                    robotState = States.IDLE;
                }
                break;
        }

        // 3. Update previous button states
        prevA1 = a1Pressed;
        prevB2 = b2Pressed;
        prevA2 = a2Pressed;

        // Telemetry
        telemetry.addData("State", robotState);
        telemetry.addData("Launcher Velocity", topLauncher.getVelocity());
        telemetry.update();
    }

    public void FieldCentricDrive(boolean isVisualServoing) {
        if (!isVisualServoing) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.guide) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            double multiplier = gamepad1.left_bumper ? 0.5 : 1.0;

            frontLeft.setPower(frontLeftPower * multiplier);
            backLeft.setPower(backLeftPower * multiplier);
            frontRight.setPower(frontRightPower * multiplier);
            backRight.setPower(backRightPower * multiplier);
        }
        // If isVisualServoing is true, this method does nothing, chassis is handled by visualServoing.visualServo()
    }

    @Override
    public void stop() {
        limelight.stop();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        topLauncher.setPower(0);
        bottomLauncher.setPower(0);
        frontIntake.setPower(0);
        backLeftServo.setPower(0);
        backRightServo.setPower(0);
    }
}