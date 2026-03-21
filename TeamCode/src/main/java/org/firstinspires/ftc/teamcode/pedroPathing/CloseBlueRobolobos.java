package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous()
public class CloseBlueRobolobos extends OpMode {

    private ElapsedTime actionTimer = new ElapsedTime();

    DcMotorEx lShooter, rShooter;
    DcMotorEx intake, index;

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Start pose — same as the full auto
    private final Pose startPose = new Pose(25.5, 125.585, Math.toRadians(180));

    // Shoot pose — straight back, y stays fixed, x moves back
    private final Pose shootPose = new Pose(49.0, 125.585, Math.toRadians(180));

    private Path StartToShoot;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("shooter velo", rShooter.getVelocity());
        telemetry.update();
    }

    public void buildPaths() {
        StartToShoot = new Path(new BezierLine(startPose, shootPose));
        // Constant 180° — no rotation
        StartToShoot.setConstantHeadingInterpolation(Math.toRadians(180));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Spin up shooter and drive straight back to shoot position
                setShooterVel(1175);
                intakePassiveIndex();
                follower.followPath(StartToShoot);
                setPathState(1);
                break;

            case 1:
                // Wait for path, then dump all 3 preloads
                if (!follower.isBusy()) {
                    magDump(1.5);
                    setPathState(2);
                }
                break;

            case 2:
                // Kill everything
                lShooter.setVelocity(0);
                rShooter.setVelocity(0);
                intake.setPower(0);
                index.setPower(0);
                index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                setPathState(3);
                break;

            case 3:
                // Terminal — do nothing
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setShooterVel(double vel) {
        lShooter.setVelocity(vel-132);
        rShooter.setVelocity(vel-132);
    }

    public void intakePassiveIndex() {
        intake.setPower(1.0);
        index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        index.setVelocity(0);
    }

    public void magDump(double seconds) {
        actionTimer.reset();
        while (actionTimer.seconds() < seconds) {
            index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            index.setPower(0.9);
            intake.setPower(0.9);
        }
    }

    void initializeHardware() {
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
    }
}