package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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

import static org.firstinspires.ftc.teamcode.pedroPathing.RedSidePedro.CloseRedSideConfigurables.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleop_Basebot;

@Autonomous()
public class RedSidePedro extends OpMode {

    // =====================================================================
    // CONSTANTS
    // =====================================================================
    public static class TeleOpConstants {
        // Shooter
        public static final double CLOSE_ZONE_VELOCITY = 1300;
        public static final double FAR_ZONE_VELOCITY = 1850;
        public static final double SHOOTER_P_GAIN = 1;
        public static final double SHOOTER_I_GAIN = 0.001;
        public static final double SHOOTER_D_GAIN = 0.0;
        public static final int SHOOTER_TOLERANCE = 10;

        // Intake
        public static final double INTAKE_POWER = 1.0;
        public static final double INTAKE_REVERSE_POWER = -0.7;
        public static final double TRIGGER_THRESHOLD = 0.2;

        // Index
        public static final int INDEX_STEP = 280;
        public static final double PASSIVE_INDEX_VELOCITY = 20;

        // Limelight
        public static final double LIMELIGHT_MOUNT_ANGLE = 12.0;
        public static final double LIMELIGHT_HEIGHT = 13.4;
        public static final double GOAL_HEIGHT = 38.75 - 9.25;

        // Pinpoint
        public static final double PINPOINT_X_OFFSET = -107.31371;
        public static final double PINPOINT_Y_OFFSET = 0.0;

        // AutoShoot regression data
        public static final double[] DISTANCES = {};
        public static final double[] VELOCITIES = {};
        public static final int REGRESSION_DEGREE = 2;

        // Distance sensor
        public static final double ACTIVATION_DISTANCE = 2; //INCHES
    }

    // =====================================================================
    // INSTANCE VARIABLES
    // =====================================================================
    private ElapsedTime actiontime = new ElapsedTime();
    double  shooterTargetVel;


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
    private final Pose startPose = new Pose(118.5, 126.585, Math.toRadians(0));

    // Generated paths
    private Path StartToShoot, IntakeCloseLine, ShootCloseLine, PrepIntakeMidLine, IntakeMidLine, ShootMidLine, PrepIntakeFarLine, IntakeFarLine, ShootFarLine;

    @Configurable
    public static class CloseRedSideConfigurables {
        //Adjustable power of dt when intaking and when not intaking (slower for ++accuracy)
        public static double intakePathMaxDrivetrainPower = 0.5;
        public static double defaultPathMaxDrivetrainPower = 0.8;

        //x coordinate of shooting pos and end of intake pos (for every line)
        public static double shootPositionXCoordinate = 100.000;
        public static double intakePathEndXCoordinate = 130.000;

        public static double shooterVelocityPreload = 1250;
        public static double shooterVelocityGoal = 1250;
        public static double shooterVelocityMid = 1250;
        public static double shooterVelocityLoadingZone = 1250;
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

        telemetry.update();
    }

    /**
     * Builds all the paths for the autonomous routine.
     */
    public void buildPaths() {
        StartToShoot = new Path(new BezierLine(startPose, new Pose(shootPositionXCoordinate, 85.000)));
        StartToShoot.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45));

        IntakeCloseLine = new Path(new BezierLine(new Pose(shootPositionXCoordinate, 85.000), new Pose(intakePathEndXCoordinate, 85.000)));
        IntakeCloseLine.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .2,
                                HeadingInterpolator.linear(Math.toRadians(45), Math.toRadians(0))
                        )
                )
        );

        ShootCloseLine = new Path(new BezierLine(new Pose(intakePathEndXCoordinate, 85.000), new Pose(shootPositionXCoordinate, 85.000)));
        ShootCloseLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45));

        PrepIntakeMidLine = new Path(new BezierLine(new Pose(shootPositionXCoordinate, 85.000), new Pose(shootPositionXCoordinate, 60.000)));
        PrepIntakeMidLine.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .2,
                                HeadingInterpolator.linear(Math.toRadians(45), Math.toRadians(0))
                        )
                )
        );

        IntakeMidLine = new Path(new BezierLine(new Pose(shootPositionXCoordinate, 60.000), new Pose(intakePathEndXCoordinate, 60.000)));
        IntakeMidLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        ShootMidLine = new Path(new BezierLine(new Pose(intakePathEndXCoordinate, 60.000), new Pose(shootPositionXCoordinate, 85.000)));
        ShootMidLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45));

        PrepIntakeFarLine = new Path(new BezierLine(new Pose(shootPositionXCoordinate, 85.000), new Pose(shootPositionXCoordinate, 35.000)));
        PrepIntakeFarLine.setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .2,
                                HeadingInterpolator.linear(Math.toRadians(45), Math.toRadians(0))
                        )
                )
        );

        IntakeFarLine = new Path(new BezierLine(new Pose(shootPositionXCoordinate, 35.000), new Pose(intakePathEndXCoordinate, 35.000)));
        IntakeFarLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        ShootFarLine = new Path(new BezierLine(new Pose(intakePathEndXCoordinate, 35.000), new Pose(shootPositionXCoordinate, 85.000)));
        ShootFarLine.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45));
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
                // Wait until robot reaches shoot position, then start intake close line
                if (!follower.isBusy()) {
                    magDump(1.0);
                    follower.setMaxPower(intakePathMaxDrivetrainPower);
                    intakePassiveIndex();
                    follower.followPath(IntakeCloseLine, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Wait until robot reaches intake close line position, then shoot
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocityGoal);
                    follower.followPath(ShootCloseLine, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Wait until robot reaches shoot position, then prep for mid line intake
                if (!follower.isBusy()) {
                    magDump(1.0);
                    follower.followPath(PrepIntakeMidLine, true);
                    setPathState(4);
                }
                break;
            case 4:
                // Wait until robot reaches prep position, then start mid line intake
                if (!follower.isBusy()) {
                    intakePassiveIndex();
                    follower.setMaxPower(intakePathMaxDrivetrainPower);
                    follower.followPath(IntakeMidLine, true);
                    setPathState(5);
                }
                break;
            case 5:
                // Wait until robot reaches intake mid line position, then shoot
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocityMid);
                    follower.followPath(ShootMidLine, true);
                    setPathState(6);
                }
                break;
            case 6:
                // Wait until robot reaches shoot position, then prep for far line intake
                if (!follower.isBusy()) {
                    magDump(1.0);
                    follower.followPath(PrepIntakeFarLine, true);
                    setPathState(7);
                }
                break;
            case 7:
                // Wait until robot reaches prep position, then start far line intake
                if (!follower.isBusy()) {
                    intakePassiveIndex();
                    follower.setMaxPower(intakePathMaxDrivetrainPower);
                    follower.followPath(IntakeFarLine, true);
                    setPathState(8);
                }
                break;
            case 8:
                // Wait until robot reaches intake far line position, then shoot
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocityLoadingZone);
                    follower.followPath(ShootFarLine, true);
                    setPathState(9);
                }
                break;
            case 9:
                // Wait until robot reaches shoot position, then end
                if (!follower.isBusy()) {
                    // Set the state to a case we won't use, so it just stops running new paths
                    magDump(1.0);
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
        while (actiontime.seconds()<seconds) {
            index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            index.setPower(1);
            intake.setPower(1);
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
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(TeleOpConstants.PINPOINT_X_OFFSET, TeleOpConstants.PINPOINT_Y_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

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
