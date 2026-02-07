package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarRedSide.FarRedSideConfigurables.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleop_Basebot;

@Autonomous()
public class FarRedSide extends OpMode {
    // =====================================================================
    // INSTANCE VARIABLES
    // =====================================================================
    private ElapsedTime actiontime = new ElapsedTime();
    double shooterTargetVel;


    // Hardware
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx lShooter, rShooter;
    DcMotorEx intake, index;
    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    DistanceSensor distance;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Start pose of the robot
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));

    // Generated paths
    private Path StartToShoot, PrepIntakeFarLine, IntakeFarLine, ShootFarLine, PrepIntakeLoadingZone, IntakeLoadingZone, ShootLoadingZone, Park;

    // =====================================================================
    // CONSTANTS
    // =====================================================================
    public static class TeleOpConstants {
        // Shooter
        public static final int SHOOTER_TOLERANCE = 10;

        // Intake
        public static final double INTAKE_POWER = 1.0;

        // Index
        public static final int INDEX_STEP = 280;
        public static final double PASSIVE_INDEX_VELOCITY = 20;
        public static final double MAG_DUMP_POWER = 0.65;

        // Limelight
        public static final double LIMELIGHT_MOUNT_ANGLE = 12.0;
        public static final double LIMELIGHT_HEIGHT = 13.4;
        public static final double GOAL_HEIGHT = 38.75 - 9.25;

        // Distance sensor
        public static final double ACTIVATION_DISTANCE = 2; //INCHES
    }

    @Configurable
    public static class FarRedSideConfigurables {
        //Adjustable power of dt when intaking and when not intaking (slower for ++accuracy)
        public static double intakePathMaxDrivetrainPower = 0.5;
        public static double defaultPathMaxDrivetrainPower = 0.8;

        //x coordinate of shooting pos and end of intake pos (for every line)
        public static double shootPositionXCoordinate = 90.000;
        public static double shootPositionTheta = 62; //Degrees
        public static double intakePathEndXCoordinate = 135;

        public static double shooterVelocityPreload = 1450;
        public static double shooterVelocitySpikeMarks = 1470;
        public static double shooterVelocityLoadingZone = 1470;

        public static double txOffsetDegrees = 3;
    }

    /**
     * This method is called once at the init of the OpMode.
     */
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setStartingPose(startPose);
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     */
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     */
    @Override
    public void start() {
        setPathState(0);
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     */
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // --- BLINKIN ---
        if (shooterTargetVel > 500 &&
                (lShooter.getVelocity() < 50 || rShooter.getVelocity() < 50)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else if (!shooterWithinTolerance(shooterTargetVel)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        } else if (shooterWithinTolerance(Teleop_Basebot.Constants.CLOSE_ZONE_VELOCITY)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (shooterWithinTolerance(Teleop_Basebot.Constants.FAR_ZONE_VELOCITY)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (withinDistance()) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path completion %", follower.getPathCompletion());
        telemetry.addData("shooter velo", rShooter.getVelocity());

        telemetry.update();
    }

    /**
     * Builds all the paths for the autonomous routine.
     */
    public void buildPaths() {
        StartToShoot = new Path(new BezierLine(startPose, new Pose(shootPositionXCoordinate, 10.000)));
        StartToShoot.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(shootPositionTheta));

        PrepIntakeFarLine = new Path(new BezierLine(new Pose(shootPositionXCoordinate, 12.000), new Pose(100, 35.000)));
        PrepIntakeFarLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        IntakeFarLine = new Path(new BezierLine(new Pose(100, 35.000), new Pose(intakePathEndXCoordinate, 35.000)));
        IntakeFarLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        ShootFarLine = new Path(new BezierLine(new Pose(intakePathEndXCoordinate, 35.000), new Pose(shootPositionXCoordinate, 12.000)));
        ShootFarLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(shootPositionTheta));

        PrepIntakeLoadingZone = new Path(new BezierCurve(
                new Pose(shootPositionXCoordinate, 12.000),
                new Pose(108.910, 58.744),
                new Pose(136.000, 28.000)
        ));
        PrepIntakeLoadingZone.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90));

        IntakeLoadingZone = new Path(new BezierLine(new Pose(136.000, 28.000), new Pose(136.000, 10.000)));
        IntakeLoadingZone.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90));

        ShootLoadingZone = new Path(new BezierCurve(
                new Pose(136.000, 9.000),
                new Pose(114.628, 40.029),
                new Pose(shootPositionXCoordinate, 12.000)
        ));
        ShootLoadingZone.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(shootPositionTheta));

        Park = new Path(new BezierLine(new Pose(shootPositionXCoordinate, 12.000), new Pose(120, 12.000)));
    }

    /**
     * Manages the path states using a Finite State Machine (FSM).
     * The switch is called continuously and runs the pathing.
     * Every time the switch changes case, it will reset the timer.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start to shoot position
                follower.followPath(StartToShoot);
                setShooterVel(shooterVelocityPreload);
                setPathState(1);
                break;
            case 1:
                // Wait until robot reaches shoot position, then prep for far line intake
                if (!follower.isBusy()) {
                    autoAlignTimeout(0.5);
                    fieldCentric(0, 0, 0, 0);
                    sleep(3000);
                    magDump(1.5);
                    follower.followPath(PrepIntakeFarLine, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Wait until robot reaches prep position, then start far line intake
                if (!follower.isBusy()) {
                    intakePassiveIndex();
                    follower.setMaxPower(intakePathMaxDrivetrainPower);
                    follower.followPath(IntakeFarLine, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Wait until robot reaches intake far line position, then shoot
                if (!follower.isBusy()) {
                    sleep(200);
                    intake.setPower(1.0);
                    follower.setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocitySpikeMarks);
                    follower.followPath(ShootFarLine, true);
                    setPathState(4);
                }
                break;
            case 4:
                // Wait until robot reaches prep position, then start mid line intake
                if (!follower.isBusy()) {
                    autoAlignTimeout(0.5);
                    fieldCentric(0, 0, 0, 0);
                    magDump(1.5);
                    follower.followPath(PrepIntakeLoadingZone, true);
                    setPathState(5);
                }
                break;
            case 5:
                // Wait until robot reaches intake mid line position, then shoot
                if (!follower.isBusy()) {
                    intakePassiveIndex();
                    follower.setMaxPower(0.3);
                    follower.followPath(IntakeLoadingZone, true);
                    setPathState(6);
                }
                break;
            case 6:
                // Wait until robot reaches shoot position, then prep for far line intake
                if (!follower.isBusy()) {
                    sleep(250);
                    intake.setPower(1.5);
                    follower.setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocityLoadingZone);
                    follower.followPath(ShootLoadingZone, true);
                    setPathState(7);
                }
                break;
            case 7:
                // Wait until robot reaches prep position, then start far line intake
                if (!follower.isBusy()) {
                    autoAlignTimeout(0.5);
                    fieldCentric(0, 0, 0, 0);
                    magDump(1.5);
                    follower.followPath(Park, true);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * Changes the states of the paths. It will also reset the timers of the individual switches.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // =====================================================================
    // SHOOTER METHODS
    // =====================================================================
    public void setShooterVel(double vel) {
        shooterTargetVel = vel;
        lShooter.setVelocity(vel);
        rShooter.setVelocity(vel);
    }

    public int getAvgShooterVel() {
        return (int) (lShooter.getVelocity() + rShooter.getVelocity() / 2);
    }

    public boolean shooterWithinTolerance(double target) {
        return withinTolerance(getAvgShooterVel(), target, TeleOpConstants.SHOOTER_TOLERANCE);
    }

    public boolean withinTolerance(double val, double target, double tol) {
        return Math.abs(target - val) <= tol;
    }

    // =====================================================================
    // INDEX METHODS
    // =====================================================================
    public void setIndexPos(int pos) {
        index.setTargetPosition(pos);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        index.setPower(1);
    }

    public boolean withinDistance() {
        return distance.getDistance(DistanceUnit.INCH) < TeleOpConstants.ACTIVATION_DISTANCE;
    }

    public void intakePassiveIndex() {
        intake.setPower(TeleOpConstants.INTAKE_POWER);
        if (!withinDistance()) {
            index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            index.setVelocity(TeleOpConstants.PASSIVE_INDEX_VELOCITY);
        } else {
            index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void magDump(double seconds) {
        actiontime.reset();
        while (actiontime.seconds() < seconds) {
            index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            index.setPower(TeleOpConstants.MAG_DUMP_POWER);
            intake.setPower(TeleOpConstants.MAG_DUMP_POWER);
        }
    }

    public void autoAlignTimeout(double seconds) {
        actiontime.reset();
        while (actiontime.seconds() < seconds) {
            autoAlign(0.25);
        }
    }

    // =====================================================================
    // LIMELIGHT METHODS
    // =====================================================================
    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angle = Math.toRadians(TeleOpConstants.LIMELIGHT_MOUNT_ANGLE + ty);
            return (TeleOpConstants.GOAL_HEIGHT - TeleOpConstants.LIMELIGHT_HEIGHT) / Math.tan(angle);
        }
        return Double.POSITIVE_INFINITY;
    }

    void setPattern(RevBlinkinLedDriver.BlinkinPattern p) {
        pattern = p;
        blinkinLedDriver.setPattern(pattern);
    }

    // =====================================================================
    // DRIVE METHODS
    // =====================================================================
    public void fieldCentric(double y, double x, double rx, double heading) {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX = rotX * Teleop_Basebot.Constants.STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double FLPower = (rotY + rotX + rx) / denominator;
        double FRPower = (rotY - rotX - rx) / denominator;
        double BLPower = (rotY - rotX + rx) / denominator;
        double BRPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(FLPower);
        frontRight.setPower(FRPower);
        backLeft.setPower(BLPower);
        backRight.setPower(BRPower);
    }

    public String autoAlign(double tolerance) {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return "RESULT NULL";
        } else if (!result.isValid()) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return "RESULT INVALID";
        }

        double tx = result.getTx() - txOffsetDegrees;
        if (Math.abs(tx) <= tolerance) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return "SUCCESS";
        }

        double pivot = Math.max(-1.0, Math.min(1.0, Teleop_Basebot.Constants.ALIGN_P_GAIN * tx));

        fieldCentric(0, 0, pivot, 0);

        return "IN PROGRESS";
    }

    void initializeHardware() {
        // --- Initialize Hardware ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        index = hardwareMap.get(DcMotorEx.class, "transfer");
        index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        index.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        index.setTargetPosition(0);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lShooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        rShooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        distance = hardwareMap.get(DistanceSensor.class, "distance");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }
}
