/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class HuskyAutoBase extends LinearOpMode {
    /* Declare OpMode members. */
    HuskyBot huskyBot = new HuskyBot();
    ElapsedTime runtime = new ElapsedTime();

    // autonomous mode constants
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 11.80;

    // 75 mm mecanum wheels
    static final double WHEEL_DIAMETER_INCHES = 75 / 25.4;
    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double COUNTS_PER_INCH = COUNTS_PER_WHEEL_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double AUTO_DRIVE_SPEED = 0.5;
    public static final double AUTO_TURN_SPEED = 0.5;
    public static final double AUTO_STRAFE_SPEED = 0.5;

    private OpenCvWebcam webcam;

    DeliveryLevelPipeline pipeline = new DeliveryLevelPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        huskyBot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    /*
     *  Method to perform a straight line forward/backward move, based on encoder counts.
     *  forward is +ve target, backward is -ve
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double distanceInches, double timeoutSecs) {
        resetDriveEncoders();
        // Determine new target position, and pass to motor controller
        // target is same for all motors
        int target = (int) (distanceInches * COUNTS_PER_INCH);
        driveToTarget(speed, target, target, target, target, timeoutSecs);
    }

    /*
     *  Method to perform a strafe move, based on encoder counts.
     *  right (looking from behind of robot) is +ve, left is -ve
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderStrafe(double speed, double distanceInches, double timeoutSecs) {
        resetDriveEncoders();
        // Determine new target position, and pass to motor controller
        int target = (int) (distanceInches * COUNTS_PER_INCH);
        driveToTarget(speed, (int) (target * 1.2), -target, -target, target, timeoutSecs);
    }

    private void resetDriveEncoders() {
        huskyBot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        huskyBot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        huskyBot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        huskyBot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void driveToTarget(double speed, int frontLeftTarget, int frontRightTarget,
                               int rearLeftTarget, int rearRightTarget, double timeoutSecs) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            huskyBot.frontLeftDrive.setTargetPosition(frontLeftTarget);
            huskyBot.frontRightDrive.setTargetPosition(frontRightTarget);
            huskyBot.rearLeftDrive.setTargetPosition(rearLeftTarget);
            huskyBot.rearRightDrive.setTargetPosition(rearRightTarget);

            // Turn On RUN_TO_POSITION
            huskyBot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            huskyBot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            huskyBot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            huskyBot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            huskyBot.frontLeftDrive.setPower(speed);
            huskyBot.frontRightDrive.setPower(speed);
            huskyBot.rearLeftDrive.setPower(speed);
            huskyBot.rearRightDrive.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSecs) &&
                    (huskyBot.frontLeftDrive.isBusy() && huskyBot.frontRightDrive.isBusy())) {
                telemetry.addData("Target Position", "front left: %7d, front right: %7d",
                        frontLeftTarget, frontRightTarget);
                telemetry.addData("Target Position", "rear left: %7d, rear right: %7d",
                        rearLeftTarget, rearRightTarget);
                displayTelemetry();
            }

            // Stop all motion;
            huskyBot.frontLeftDrive.setPower(0);
            huskyBot.frontRightDrive.setPower(0);
            huskyBot.rearLeftDrive.setPower(0);
            huskyBot.rearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            huskyBot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            huskyBot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            huskyBot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            huskyBot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private void displayTelemetry() {
        telemetry.addData("Position", "front left: %7d, front right: %7d",
                huskyBot.frontLeftDrive.getCurrentPosition(),
                huskyBot.frontRightDrive.getCurrentPosition());
        telemetry.addData("Position", "rear left: %7d, rear right: %7d",
                huskyBot.rearLeftDrive.getCurrentPosition(),
                huskyBot.rearRightDrive.getCurrentPosition());
        telemetry.addData("Velocity", "front left (%.2f), front right (%.2f)",
                huskyBot.frontLeftDrive.getVelocity(),
                huskyBot.frontRightDrive.getVelocity());
        telemetry.addData("Velocity", "rear left (%.2f), rear right (%.2f)",
                huskyBot.rearLeftDrive.getVelocity(),
                huskyBot.rearRightDrive.getVelocity());
        telemetry.addData("Power", "front left (%.2f), front right (%.2f)",
                huskyBot.frontLeftDrive.getPower(),
                huskyBot.frontRightDrive.getPower());
        telemetry.addData("Power", "rear left (%.2f), rear right (%.2f)",
                huskyBot.rearLeftDrive.getPower(),
                huskyBot.rearRightDrive.getPower());

        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();

        telemetry.update();
    }

    public static class DeliveryLevelPipeline extends OpenCvPipeline {

        public enum DeliveryLevel {
            LEVEL_0,
            LEVEL_1,
            LEVEL_2,
            LEVEL_3
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(10, 100);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(130, 100);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(250, 100);
        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 70;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile DeliveryLevel deliveryLevel = DeliveryLevel.LEVEL_0;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if (max == avg1) // Was it from region 1?
            {
                deliveryLevel = DeliveryLevel.LEVEL_1; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg2) // Was it from region 2?
            {
                deliveryLevel = DeliveryLevel.LEVEL_2; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg3) // Was it from region 3?
            {
                deliveryLevel = DeliveryLevel.LEVEL_3; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public DeliveryLevel getDeliveryLevel() {
            return deliveryLevel;
        }
    }
}
