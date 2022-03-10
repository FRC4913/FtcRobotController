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

import static org.firstinspires.ftc.teamcode.HuskyAutoBase.DeliveryLevelPipeline.DeliveryLevel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name = "Auto Blue Right", group = "Auto", preselectTeleOp = "Husky TeleOpMode")
public class HuskyAuto_BlueRight extends HuskyAutoBase {

    // pre loaded cargo delivery
    public static double DISTANCE1_FORWARD_IN = 6;
    public static double DISTANCE2_STRAFE_IN = -20;
    public static double DISTANCE3_FORWARD_IN;
    public static double DISTANCE4_BACKWARD_IN = -8;

    // drive to carousel and duck delivery
    public static double TURN_ANGLE = -90;
    public static double DISTANCE5_BACKWARD_IN = -32;
    public static double SENSOR_DISTANCE6_IN = 4;
    public static double DISTANCE6_STRAFE_IN = -12;
    public static double DISTANCE7_STRAFE_IN = 4;

    public static double CAROUSEL_ROTATE_POWER = 0.5;

    // storage unit parking
    public static double STRAFE_DISTANCE9_IN = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        telemetry.addData("Mode", this.getClass().getSimpleName());
        telemetry.update();

        DeliveryLevel deliveryLevel = DeliveryLevel.LEVEL_0;    // default undefined level

        waitForStart();
        runtime.reset();

        // initial wait time if needed
        while (opModeIsActive() && (runtime.seconds() < INIT_WAIT_SECS)) {
        }

        runtime.reset();

        // get the delivery level with timeout of 5 seconds. if the delivery level isn't detected
        // within timeout, it defaults to level 1 (see switch statement below)
        while (opModeIsActive() && (runtime.seconds() < 5))
        {
            deliveryLevel = pipeline.getDeliveryLevel();
            if (deliveryLevel != DeliveryLevel.LEVEL_0) {
                break;
            }
            sleep(50);
        }

        telemetry.addData("Delivery", "Got delivery level " + deliveryLevel);
        telemetry.update();

        switch (deliveryLevel) {
            case LEVEL_2:
                huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_2);
                DISTANCE3_FORWARD_IN = 9.5;
                break;
            case LEVEL_3:
                huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_3);
                DISTANCE3_FORWARD_IN = 10.5;
                break;
            case LEVEL_1:
            default:
                huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_1);
                DISTANCE3_FORWARD_IN = 8.5;
                break;
        }
        huskyBot.arm.setVelocity(300);

        encoderDrive(AUTO_DRIVE_SPEED, DISTANCE1_FORWARD_IN, 1);
        encoderStrafe(AUTO_STRAFE_SPEED, DISTANCE2_STRAFE_IN, 2);
        encoderDrive(AUTO_DRIVE_SPEED, DISTANCE3_FORWARD_IN, 1);

        // deliver cargo
        runtime.reset();
        huskyBot.intaker.setPower(HuskyBot.INTAKER_POWER_OUT);
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
        }
        huskyBot.intaker.setPower(0);

        // back up before lowering arm
        encoderDrive(AUTO_DRIVE_SPEED, DISTANCE4_BACKWARD_IN, 1);
        huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_0);
        huskyBot.arm.setVelocity(300);


        // drive to carousel and deliver duck
        encoderTurn(AUTO_TURN_SPEED, TURN_ANGLE, 2);
        encoderDrive(AUTO_DRIVE_SPEED, DISTANCE5_BACKWARD_IN, 2);

        // strafe to side against the wall
        encoderStrafe(AUTO_STRAFE_SPEED/1.1, DISTANCE6_STRAFE_IN,2);
        // strafe slightly away
        encoderStrafe(AUTO_STRAFE_SPEED, DISTANCE7_STRAFE_IN,1);

        // back up to touch the spinner to carousel
        double sensorDistance = huskyBot.distanceSensorBack.getDistance(DistanceUnit.INCH);
        encoderDrive(AUTO_DRIVE_SPEED/1.2,-(sensorDistance-SENSOR_DISTANCE6_IN),1);

        carouselRotate(CAROUSEL_ROTATE_POWER,4);

        // park in storage unit
        encoderStrafe(AUTO_STRAFE_SPEED, STRAFE_DISTANCE9_IN, 2);
        sensorDistance = huskyBot.distanceSensorBack.getDistance(DistanceUnit.INCH);
        encoderDrive(AUTO_DRIVE_SPEED/1.5,-sensorDistance,2);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
