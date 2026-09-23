package org.firstinspires.ftc.team5898;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.pedroPathing.Constants;


@Autonomous(name = "PedroPathing Blue - OLD")
@Disabled
@Deprecated
public class PedroPathing_Test_BlueSide_OLD extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Flywheel hardware
    private DcMotorEx topLauncher, bottomLauncher;
    private DcMotor frontIntake;
    private CRServo backLeftServo, backRightServo;

    // Flywheel constants
    private final double LAUNCH_VELOCITY = 1220;
    private final double INTAKE_POWER = CannonConstants.IntakePower;
    private final Double P = CannonConstants.kP;
    private final Double I = CannonConstants.kI;
    private final Double D = CannonConstants.kD;
    private final Double F = CannonConstants.kF;

    // State flags to prevent repeated calls
    private boolean launchersStarted = false;
    private boolean intakeStarted = false;

    public enum PathState {
        // START POSITION - END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > SCORING
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_FIRSTROW,
        DRIVE_FIRSTROW_GRABROW1,
        SHOOT_ROW1,
        DRIVE_GRABROW1_SHOOTPOS,

        DRIVE_SHOOTPOS_SECONDROW,
        DRIVE_SECONDROW_GRABROW2,
        DRIVE_GRABROW2_SHOOTPOS,
        SHOOT_ROW2,

        DRIVE_SHOOTPOS_THIRDROW,
        DRIVE_THIRDROW_GRABROW3,
        DRIVE_GRABROW3_SHOOTPOS,
        SHOOT_ROW3,

        DRIVE_SHOOTPOS_ENDPOS,
        IDLE
    }

    PathState pathState;

    private final Pose startPose = new Pose(22, 126, Math.toRadians(143));
    private final Pose shootPose = new Pose(46, 102, Math.toRadians(135));
    private final Pose firstRowPose = new Pose(62, 64, Math.toRadians(180));
    private final Pose firstGrabPose = new Pose(55, 64, Math.toRadians(180));
    private final Pose secondRowPose = new Pose(62, 39, Math.toRadians(180));
    private final Pose secondGrabPose = new Pose(49, 39, Math.toRadians(180));
    private final Pose thirdRowPose = new Pose(62, 15, Math.toRadians(180));
    private final Pose thirdGrabPose = new Pose(49, 15, Math.toRadians(180));
    private final Pose endPose = new Pose(68, 126, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;
    private PathChain driveShootPosRow1Pos, driveRow1PosGrab1Pos, driveGrab1PosShootPos;
    private PathChain driveShootPosRow2Pos, driveRow2PosGrab2Pos, driveGrab2PosShootPos;
    private PathChain driveShootPosRow3Pos, driveRow3PosGrab3Pos, driveGrab3PosShootPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosRow1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstRowPose.getHeading())
                .build();
        driveRow1PosGrab1Pos = follower.pathBuilder()
                .addPath(new BezierLine(firstRowPose, firstGrabPose))
                .setLinearHeadingInterpolation(firstRowPose.getHeading(), firstGrabPose.getHeading())
                .build();
        driveGrab1PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(firstGrabPose, shootPose))
                .setLinearHeadingInterpolation(firstGrabPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosRow2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondRowPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondRowPose.getHeading())
                .build();
        driveRow2PosGrab2Pos = follower.pathBuilder()
                .addPath(new BezierLine(secondRowPose, secondGrabPose))
                .setLinearHeadingInterpolation(secondRowPose.getHeading(), secondGrabPose.getHeading())
                .build();
        driveGrab2PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(secondGrabPose, shootPose))
                .setLinearHeadingInterpolation(secondGrabPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosRow3Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdRowPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdRowPose.getHeading())
                .build();
        driveRow3PosGrab3Pos = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowPose, thirdGrabPose))
                .setLinearHeadingInterpolation(thirdRowPose.getHeading(), thirdGrabPose.getHeading())
                .build();
        driveGrab3PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdGrabPose, shootPose))
                .setLinearHeadingInterpolation(thirdGrabPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!launchersStarted) {
                    startLaunchers();
                    launchersStarted = true;
                    pathTimer.resetTimer();
                    telemetry.addLine("Spooling up launchers...");
                }

                if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.5 && !intakeStarted) {
                    setIntakePower(INTAKE_POWER);
                    intakeStarted = true;
                    telemetry.addLine("Firing preload!");
                }

                if (pathTimer.getElapsedTimeSeconds() > 7) {
                    stopLaunchers();
                    setIntakePower(0);
                    launchersStarted = false;
                    intakeStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_FIRSTROW);
                }
                break;

            case DRIVE_SHOOTPOS_FIRSTROW:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosRow1Pos);
                    setPathState(PathState.DRIVE_FIRSTROW_GRABROW1);
                }
                break;

            case DRIVE_FIRSTROW_GRABROW1:
                if (!follower.isBusy()) {
                    setIntakePower(INTAKE_POWER);
                    follower.followPath(driveRow1PosGrab1Pos);
                    setPathState(PathState.DRIVE_GRABROW1_SHOOTPOS);
                }
                break;

            case DRIVE_GRABROW1_SHOOTPOS:
                if (!follower.isBusy()) {
                    setIntakePower(0);
                    follower.followPath(driveGrab1PosShootPos);
                    setPathState(PathState.SHOOT_ROW1);
                }
                break;

            case SHOOT_ROW1:
                if (!launchersStarted) {
                    startLaunchers();
                    launchersStarted = true;
                    pathTimer.resetTimer();
                    telemetry.addLine("Spooling up launchers...");
                }

                if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.5 && !intakeStarted) {
                    setIntakePower(INTAKE_POWER);
                    intakeStarted = true;
                    telemetry.addLine("Firing row 1 sample!");
                }

                if (pathTimer.getElapsedTimeSeconds() > 7) {
                    stopLaunchers();
                    setIntakePower(0);
                    launchersStarted = false;
                    intakeStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_SECONDROW);
                }
                break;

            case DRIVE_SHOOTPOS_SECONDROW:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosRow2Pos);
                    setPathState(PathState.DRIVE_SECONDROW_GRABROW2);
                }
                break;

            case DRIVE_SECONDROW_GRABROW2:
                if (!follower.isBusy()) {
                    setIntakePower(INTAKE_POWER);
                    follower.followPath(driveRow2PosGrab2Pos);
                    setPathState(PathState.DRIVE_GRABROW2_SHOOTPOS);
                }
                break;

            case DRIVE_GRABROW2_SHOOTPOS:
                if (!follower.isBusy()) {
                    setIntakePower(0);
                    follower.followPath(driveGrab2PosShootPos);
                    setPathState(PathState.SHOOT_ROW2);
                }
                break;

            case SHOOT_ROW2:
                if (!launchersStarted) {
                    startLaunchers();
                    launchersStarted = true;
                    pathTimer.resetTimer();
                    telemetry.addLine("Spooling up launchers...");
                }

                if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.5 && !intakeStarted) {
                    setIntakePower(INTAKE_POWER);
                    intakeStarted = true;
                    telemetry.addLine("Firing row 2 sample!");
                }

                if (pathTimer.getElapsedTimeSeconds() > 7) {
                    stopLaunchers();
                    setIntakePower(0);
                    launchersStarted = false;
                    intakeStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_THIRDROW);
                }
                break;

            case DRIVE_SHOOTPOS_THIRDROW:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosRow3Pos);
                    setPathState(PathState.DRIVE_THIRDROW_GRABROW3);
                }
                break;

            case DRIVE_THIRDROW_GRABROW3:
                if (!follower.isBusy()) {
                    setIntakePower(INTAKE_POWER);
                    follower.followPath(driveRow3PosGrab3Pos);
                    setPathState(PathState.DRIVE_GRABROW3_SHOOTPOS);
                }
                break;

            case DRIVE_GRABROW3_SHOOTPOS:
                if (!follower.isBusy()) {
                    setIntakePower(0);
                    follower.followPath(driveGrab3PosShootPos);
                    setPathState(PathState.SHOOT_ROW3);
                }
                break;

            case SHOOT_ROW3:
                if (!launchersStarted) {
                    startLaunchers();
                    launchersStarted = true;
                    pathTimer.resetTimer();
                    telemetry.addLine("Spooling up launchers...");
                }

                if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.5 && !intakeStarted) {
                    setIntakePower(INTAKE_POWER);
                    intakeStarted = true;
                    telemetry.addLine("Firing row 3 sample!");
                }

                if (pathTimer.getElapsedTimeSeconds() > 7) {
                    stopLaunchers();
                    setIntakePower(0);
                    launchersStarted = false;
                    intakeStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosEndPos);
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                telemetry.addLine("Auto complete!");
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize flywheel motors
        topLauncher = hardwareMap.get(DcMotorEx.class, "TLaunch");
        bottomLauncher = hardwareMap.get(DcMotorEx.class, "BLaunch");
        topLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to use encoders for velocity control
        topLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for velocity control
        topLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));
        bottomLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));

        // Set zero power behavior to FLOAT for launchers (reduces resistance)
        topLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize intake system
        frontIntake = hardwareMap.get(DcMotor.class, "Intake");
        backLeftServo = hardwareMap.get(CRServo.class, "LServo");
        backRightServo = hardwareMap.get(CRServo.class, "RServo");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.FORWARD);

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    /**
     * Start the flywheel launchers at the configured velocity
     */
    private void startLaunchers() {
        topLauncher.setVelocity(-LAUNCH_VELOCITY);
        bottomLauncher.setVelocity(-LAUNCH_VELOCITY);
    }

    /**
     * Stop the flywheel launchers
     */
    private void stopLaunchers() {
        topLauncher.setPower(0);
        bottomLauncher.setPower(0);
    }

    /**
     * Set power to intake system (front motor and back servos)
     * @param power Power value for intake system
     */
    private void setIntakePower(double power) {
        frontIntake.setPower(power);
        backLeftServo.setPower(power);
        backRightServo.setPower(power);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("isBusy", follower.isBusy());
    }
}
