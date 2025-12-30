package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Project1Hardware;

import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Mecanum drive control with field-centric and autonomous movement support.
 *
 * @author FTC Team 23070 Royal Turtles
 */
public class MecanumDrive {
    Project1Hardware robot;
    double max, sin, cos, theta, power, vertical, horizontal, pivot, heading;
    double FLPower, FRPower, BLPower, BRPower;


    private double alignIntegral = 0;
    private double alignLastError = 0;
    private long alignLastTime = 0;

    public MecanumDrive(Project1Hardware robot) {
        this.robot = robot;
    }

    /**
     * Field-centric drive with heading compensation.
     * Equivalent to {@link #remote2(double, double, double, double)} but uses hardware motor directions.
     *
     * @param vertical   Vertical input (-1.0 to 1.0)
     * @param horizontal Horizontal input (-1.0 to 1.0)
     * @param pivot      Rotation input (-1.0 to 1.0)
     * @param heading    Current robot heading in radians
     */
    public void remote(double vertical, double horizontal, double pivot, double heading) {
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading;

        theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
        power = Math.hypot(horizontal, vertical);

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        FLPower = power * (cos / max) + pivot;
        FRPower = power * (sin / max) - pivot;
        BLPower = power * -(sin / max) - pivot;
        BRPower = power * -(cos / max) + pivot;

        robot.frontLeft.setPower(-FLPower);
        robot.frontRight.setPower(-FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Field-centric drive with heading compensation.
     * Equivalent to {@link #remote(double, double, double, double)} but sets motor directions programmatically.
     *
     * @param vertical   Vertical input (-1.0 to 1.0)
     * @param horizontal Horizontal input (-1.0 to 1.0)
     * @param pivot      Rotation input (-1.0 to 1.0)
     * @param heading    Current robot heading in radians
     */
    public void remote2(double vertical, double horizontal, double pivot, double heading) {
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.backRight.setDirection(DcMotor.Direction.REVERSE);

        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading + (Math.PI / 2);

        theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
        power = Math.hypot(horizontal, vertical);

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        FLPower = power * (cos / max) - pivot;
        FRPower = power * (sin / max) + pivot;
        BLPower = power * (sin / max) - pivot;
        BRPower = power * (cos / max) + pivot;

        robot.frontLeft.setPower(FLPower);
        robot.frontRight.setPower(FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Autonomous movement at specified angle with rotation control.
     *
     * @param theta Direction angle in degrees (0-360)
     * @param pivot Rotation input (-1.0 to 1.0)
     * @param power Drive power (0.0 to 1.0)
     */
    public void part1(double theta, double pivot, double power) {
        theta = 2 * Math.PI + (theta / 360 * 2 * Math.PI) - Math.PI / 2;

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        FLPower = power * (cos / max) + pivot;
        FRPower = power * sin / max - pivot;
        BLPower = power * -(sin / max) - pivot;
        BRPower = power * -(cos / max) + pivot;

        robot.frontLeft.setPower(-FLPower);
        robot.frontRight.setPower(-FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Autonomous drive for specified distance. Blocks until complete.
     *
     * @param target   Target direction angle in degrees (0-360)
     * @param power    Drive power (0.0 to 1.0)
     * @param pivot    Rotation input (-1.0 to 1.0)
     * @param distance Target distance in encoder ticks
     */
    public void drive(double target, double power, double pivot, double distance) {
        this.theta = Math.PI + (target * Math.PI / 180);
        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        int FL = robot.frontLeft.getCurrentPosition();
        int FR = robot.frontRight.getCurrentPosition();
        int BL = robot.backLeft.getCurrentPosition();
        int BR = robot.backRight.getCurrentPosition();

        double orig = FL;
        double cur = orig;

        while (Math.abs(cur - orig) <= distance) {
            FL = robot.frontLeft.getCurrentPosition();
            FR = robot.frontRight.getCurrentPosition();
            BL = robot.backLeft.getCurrentPosition();
            BR = robot.backRight.getCurrentPosition();

            cur = FL;

            FLPower = power * -(cos / max) + pivot;
            FRPower = power * sin / max + pivot;
            BLPower = power * -(sin / max) + pivot;
            BRPower = power * cos / max + pivot;

            robot.frontLeft.setPower(-FLPower);
            robot.frontRight.setPower(-FRPower);
            robot.backLeft.setPower(BLPower);
            robot.backRight.setPower(BRPower);

            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Rotates robot to align with limelight target. Blocks until aligned.
     *
     * @param tolerance Maximum acceptable tx value in degrees
     * @return true if aligned, false if no valid target
     */
    public boolean autoAlign(double tolerance) {
        LLResult result = robot.limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            stopAllDriveMotors();
            return false;
        }

        double tx = result.getTx();
        if (Math.abs(tx) <= tolerance) {
            stopAllDriveMotors();
            return true;
        }

        double error = tx;

        double proportional = BotConstants.AutoAlign.ALIGN_P_GAIN * error;

        double pivot = Math.max(-1.0, Math.min(1.0, proportional));

        robot.frontLeft.setPower(pivot);
        robot.frontRight.setPower(pivot);
        robot.backLeft.setPower(pivot);
        robot.backRight.setPower(pivot);

        return false;
    }

    /**
     * Stop all drive motors with brake mode
     */
    public void stopAllDriveMotors() {
        robot.frontLeft.setPower(-FLPower);
        robot.frontRight.setPower(-FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}