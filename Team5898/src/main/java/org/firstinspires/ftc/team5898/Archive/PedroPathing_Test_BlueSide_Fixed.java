package org.firstinspires.ftc.team5898;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.pedroPathing.Constants;

@Autonomous(name = "PedroPathing Blue - Fixed",preselectTeleOp = "Beta TeleOP (PID)")
public class PedroPathing_Test_BlueSide_Fixed extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Flywheel hardware
    private DcMotorEx topLauncher, bottomLauncher;
    private DcMotor frontIntake;
    private CRServo backLeftServo, backRightServo;
    private Servo stopperServo;

    // Flywheel constants
    private final double LAUNCH_VELOCITY = CannonConstants.FRONT_LAUNCH_VELOCITY;
    private final double INTAKE_POWER = CannonConstants.IntakePower;
    private final Double P = CannonConstants.kP;
    private final Double I = CannonConstants.kI;
    private final Double D = CannonConstants.kD;
    private final Double F = CannonConstants.kF;

    // State flags to prevent repeated calls
    private boolean launchersStarted = false;
    private boolean intakeStarted = false;
    private boolean shootSequenceStarted = false;

    public enum PathState {
        // START POSITION - END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > SCORING
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_PPG,
        DRIVE_PPG_GRABPPG,
        DRIVE_GRABPPG_SHOOTPOS,
        SHOOT_PPG,

        DRIVE_SHOOTPOS_PGP,
        DRIVE_PGP_GRABPGP,
        DRIVE_GRABPGP_SHOOTPOS,
        SHOOT_PGP,

        DRIVE_SHOOTPOS_GPP,
        DRIVE_GPP_GRABGPP,
        DRIVE_GRABGPP_SHOOTPOS,
        SHOOT_GPP,

        DRIVE_SHOOTPOS_PARKPOS,
        IDLE
    }

    PathState pathState;
    private void runShootSequence(String label, PathState nextState){
        if (follower.isBusy()) {
            telemetry.addLine(label);
            return;}
        if (!shootSequenceStarted){
            shootSequenceStarted = true;
            launchersStarted = false;
            intakeStarted = false;

            pathTimer.resetTimer();
            startLaunchers();
            launchersStarted = true;

            telemetry.addLine("Arrived. Spooling for: " + label);
        }

        double time = pathTimer.getElapsedTimeSeconds();
        if( time>0.5 && time < 1 && !intakeStarted){
            openStopper();
        }
        if (time > 1.5 && time < 2 && !intakeStarted) {
            setIntakePower(INTAKE_POWER);
            intakeStarted = true;
            telemetry.addLine("Firing sample!");
        }
        if (time > 5.7){
            stopLaunchers();
            setIntakePower(0);
            closeStopper();
            launchersStarted = false;
            intakeStarted = false;
            shootSequenceStarted = false;
            setPathState(nextState);
        }
    }

    private final Pose startPose = new Pose(22, 126, Math.toRadians(143));
    private final Pose shootPose = new Pose(49, 100, Math.toRadians(140));
    private final Pose PPGPose = new Pose(49, 84, Math.toRadians(180));
    private final Pose PPGGrabPose = new Pose(16, 84, Math.toRadians(180));
    private final Pose PGPPose = new Pose(49, 61, Math.toRadians(180));
    private final Pose PGPGrabPose = new Pose(11, 61, Math.toRadians(180));
//    private final Pose GPPPose = new Pose(45, 37, Math.toRadians(180));
//    private final Pose GPPGrabPose = new Pose(13, 37, Math.toRadians(180));

    private final Pose ParkPose = new Pose(33, 75, Math.toRadians(180));

    private PathChain StartToShootPose, ShootToParkPose;
    private PathChain ShootToPPGPose, GrabPPGPose, PPGToShootPose;
    private PathChain ShootToPGPPose, GrabPGPPose, PGPToShootPose;
//    private PathChain ShootToGPPPose, GrabGPPPose, GPPToShootPose;

    public void buildPaths() {
        StartToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        ShootToPPGPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, PPGPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), PPGPose.getHeading())
                .build();
        GrabPPGPose = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, PPGGrabPose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), PPGGrabPose.getHeading())
                .build();
        PPGToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(PPGGrabPose, shootPose))
                .setLinearHeadingInterpolation(PPGGrabPose.getHeading(), shootPose.getHeading())
                .build();

        ShootToPGPPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, PGPPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), PGPPose.getHeading())
                .build();
        GrabPGPPose = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, PGPGrabPose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), PGPGrabPose.getHeading())
                .build();
        PGPToShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(PGPGrabPose,
                        new Pose(62.00, 70.00),
                        shootPose))
                .setLinearHeadingInterpolation(PGPGrabPose.getHeading(), shootPose.getHeading())
                .build();

//        ShootToGPPPose = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, GPPPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), GPPPose.getHeading())
//                .build();
//        GrabGPPPose = follower.pathBuilder()
//                .addPath(new BezierLine(GPPPose, GPPGrabPose))
//                .setLinearHeadingInterpolation(GPPPose.getHeading(), GPPGrabPose.getHeading())
//                .build();
//        GPPToShootPose = follower.pathBuilder()
//                .addPath(new BezierLine(GPPGrabPose, shootPose))
//                .setLinearHeadingInterpolation(GPPGrabPose.getHeading(), shootPose.getHeading())
//                .build();

        ShootToParkPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ParkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ParkPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(StartToShootPose,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                runShootSequence("PRELOAD", PathState.DRIVE_SHOOTPOS_PPG);
                break;

            case DRIVE_SHOOTPOS_PPG:
                if (!follower.isBusy()) {
                    follower.followPath(ShootToPPGPose);
                    setPathState(PathState.DRIVE_PPG_GRABPPG);
                }
                break;

            case DRIVE_PPG_GRABPPG:
                if (!follower.isBusy()) {
                    setIntakePower(INTAKE_POWER);
                    follower.followPath(GrabPPGPose, .5, false);
                    setPathState(PathState.DRIVE_GRABPPG_SHOOTPOS);
                }
                break;

            case DRIVE_GRABPPG_SHOOTPOS:
                if (!follower.isBusy()) {
                    setIntakePower(0);
                    follower.followPath(PPGToShootPose,true);
                    launchersStarted = true;
                    startLaunchers();
                    setPathState(PathState.SHOOT_PPG);
                }
                break;

            case SHOOT_PPG:
                runShootSequence("PPG", PathState.DRIVE_SHOOTPOS_PGP);
                break;

            case DRIVE_SHOOTPOS_PGP:
                if (!follower.isBusy()) {
                    follower.followPath(ShootToPGPPose);
                    setPathState(PathState.DRIVE_PGP_GRABPGP);
                }
                break;

            case DRIVE_PGP_GRABPGP:
                if (!follower.isBusy()) {
                    setIntakePower(INTAKE_POWER);
                    follower.followPath(GrabPGPPose,0.5,false);
                    setPathState(PathState.DRIVE_GRABPGP_SHOOTPOS);
                }
                break;

            case DRIVE_GRABPGP_SHOOTPOS:
                if (!follower.isBusy()) {
                    setIntakePower(0);

                    follower.followPath(PGPToShootPose,true);
                    setPathState(PathState.DRIVE_SHOOTPOS_PARKPOS);
                }
                break;
//
//            case SHOOT_PGP:
//                runShootSequence("PGP", PathState.DRIVE_SHOOTPOS_PARKPOS);
//                break;


            case DRIVE_SHOOTPOS_PARKPOS:
                if (!follower.isBusy()) {
                    follower.followPath(ShootToParkPose);
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
        stopperServo = hardwareMap.get(Servo.class, "STPR");

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
    private void openStopper() {
        stopperServo.setPosition(CannonConstants.stopperOpenPosition);
    }
    private void closeStopper(){
        stopperServo.setPosition(CannonConstants.stopperClosePosition);
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
