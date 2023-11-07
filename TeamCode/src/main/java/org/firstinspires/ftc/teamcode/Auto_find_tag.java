/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto_find_tag", group = "Concept")

public class Auto_find_tag extends LinearOpMode {

    private DistanceSensor sensorDistance;
    ElapsedTime timmer = new ElapsedTime();
    DcMotor LB_Drive;
    DcMotor RB_Drive;
    DcMotor LF_Drive;
    DcMotor RF_Drive;
    double x_distance = 0;
    double y_distance = 0;
    double tag_id = 0;
    double tag_find = 5;
    double tag_find_2 = 6;
    double x_read = 100;
    double y_read = 100;
    double power = 0.2;

    double D_value = 0;
    boolean move_y = false;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    static final double FEET_PER_METER = 3.28084;

    @Override
    public void runOpMode() {

        initAprilTag();
        //Motor Mapping
        LB_Drive = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");

        //setting the direction of motors
        LB_Drive.setDirection(DcMotor.Direction.FORWARD);
        RB_Drive.setDirection(DcMotor.Direction.REVERSE);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the DS start button to be touched.
        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "D_Sensor");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Ready to Start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            forward_drive_time(3500,power,0);
            turnright_drive(2200,power,0);
            telemetryAprilTag();
            sleep(500);
            forward_drive_findtag(power,270);
            stop_robot();
            forward_drive_td(25, power, 270);
            tag_find = tag_find_2;
            stop_robot();
            telemetryAprilTag();
            telemetry.addLine(String.format("XYZ %6.1f  %6.1f  (inch)", x_read, y_read));
            sleep(2000);
            double offset_left = -4;
            double offset_right = -6;
            while (x_read > offset_left || x_read < offset_right) {
                telemetryAprilTag();
                telemetry.addLine(String.format("XYZ %6.1f  %6.1f  (inch)", x_read, y_read));
                telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
                // Push telemetry to the Driver Station.\
                telemetry.update();
                if (x_read > offset_left) {
                    right_drive_td(offset_left, power, 270);
                } else {
                    left_drive_td(offset_right, power, 270);
                }
                sleep(20);
            }
            stop_robot();
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.update();

            power = 0.4;
            forward_drive_td(20, power, 270);
            stop_robot();
            D_value = sensorDistance.getDistance(DistanceUnit.CM);
            power = 0.1;
            forward_drive_distance(10,power,270);
            stop_robot();
            telemetry.clear();
            telemetry.update();
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.update();
            sleep(4000);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }// end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Function to add telemetry about AprilTag detections.
     */
    void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tag_find) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                x_read = detection.ftcPose.x;
                y_read = detection.ftcPose.y;
                tag_id = detection.id;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    // Function to get angle change from gyro
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double checkDirection_90() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction = 0, angle, gain = .02;

        angle = getAngle();

        if (angle == -90);
        else
            correction = -angle + 90;        // reverse sign of angle for correction.


        correction = correction * gain;

        return correction;
    }

    private double checkDirection_270() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction = 0, angle, gain = .02;

        angle = getAngle();

        if (angle == 90);
        else
            correction = -angle - 90;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double getAngle_180() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = (angles.firstAngle) - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void forward_drive_td(double y_final, double power, double direction) {
        while (y_read > y_final) {
            telemetryAprilTag();
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void backward_drive_td(double y_final, double power, double direction) {
        while (y_read < y_final) {
            telemetryAprilTag();
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void right_drive_td(double x_final, double power, double direction) {
        while (x_read > x_final) {
            telemetryAprilTag();
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(power - correction);
        }
    }

    private void left_drive_td(double x_final, double power, double direction) {
        while (x_read < x_final) {
            telemetryAprilTag();
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void turnright_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            LB_Drive.setPower((power * -1));
            RB_Drive.setPower((power * 1));
            LF_Drive.setPower((power * -1));
            RF_Drive.setPower((power * 1));
        }
    }

    private void turnleft_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time && opModeIsActive()) {
            LB_Drive.setPower((power * 1));
            RB_Drive.setPower((power * -1));
            LF_Drive.setPower((power * 1));
            RF_Drive.setPower((power * -1));
        }
    }

    private void right_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(power - correction);
        }
    }

    private void left_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time && opModeIsActive()) {
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void forward_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time && opModeIsActive()) {
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void forward_drive_distance(double distance_s, double power, double direction) {
        D_value = sensorDistance.getDistance(DistanceUnit.CM);
        while (D_value > distance_s) {
            D_value = sensorDistance.getDistance(DistanceUnit.CM);
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void forward_drive_findtag(double power, double direction) {
        while (tag_find != tag_id) {
            telemetryAprilTag();
            if (direction == 0) {
                correction = checkDirection();
            } else if (direction == 270) {
                correction = checkDirection_270();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }
    private void stop_robot() {
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);
    }

}   // end class