/*
 * decision.c
 *
 *  Created on: Dec 18, 2024
 *      Author: user
 */
#include "stdint.h"
#include "math.h"

uint8_t decision=0;
uint8_t max_range = 10;
float decisionSensitivity=0.3;
//Forward 		- 0
//Strong Left 	- 1
//Left 			- 2
//Backward 		- 3
//Right 		- 4
//Strong Right 	- 5

uint8_t decisionLogicTesting(){
	LL_mDelay(1000);

	decision=decision+1;
	if(decision>5){
		decision=0;
	}

	return decision;
}


uint8_t decisionLogic(float distances[3][4]) {
    // Parameters
    float sensor_base_angle = 25.0f / 4.0f; // Degrees
    float sensor_tilt[] = {0.0f, -15.0f, 15.0f}; // Tilt angles
    float dir[] = {1.0f, 1.0f, 1.0f, 1.0f};
    float sensor_pos_offset[][2] = {{0.0f, 0.0f}, {-0.05f, 0.0f}, {0.05f, 0.0f}}; // Sensor positions

    float x[12] = {0.0f};
    float y[12] = {0.0f};

    // Angles for sensors
    float sensor_mid_angles[] = {
        -2 * sensor_base_angle,
        -sensor_base_angle,
        sensor_base_angle,
        2 * sensor_base_angle
    };

    float sensor_left_angles[] = {
        (-2 * sensor_base_angle + sensor_tilt[1]),
        (-sensor_base_angle + sensor_tilt[1]),
        (sensor_base_angle + sensor_tilt[1]),
        (2 * sensor_base_angle + sensor_tilt[1])
    };

    float sensor_right_angles[] = {
        (-2 * sensor_base_angle + sensor_tilt[2]),
        (-sensor_base_angle + sensor_tilt[2]),
        (sensor_base_angle + sensor_tilt[2]),
        (2 * sensor_base_angle + sensor_tilt[2])
    };

    // Calculate x and y coordinates for each sensor
    for (uint8_t i = 0; i < 4; ++i) {
        float angle = sensor_mid_angles[i] * M_PI / 180.0f;
        y[i] = cosf(angle) * distances[0][i] / max_range;
        x[i] = dir[i] * sinf(angle) * distances[0][i] / max_range + sensor_pos_offset[0][0];
    }

    for (uint8_t i = 4; i < 8; ++i) {
        float angle = sensor_left_angles[i - 4] * M_PI / 180.0f;
        y[i] = cosf(angle) * distances[1][i - 4] / max_range;
        x[i] = dir[i - 4] * sinf(angle) * distances[1][i - 4] / max_range + sensor_pos_offset[1][0];
    }

    for (uint8_t i = 8; i < 12; ++i) {
        float angle = sensor_right_angles[i - 8] * M_PI / 180.0f;
        y[i] = cosf(angle) * distances[2][i - 8] / max_range;
        x[i] = dir[i - 8] * sinf(angle) * distances[2][i - 8] / max_range + sensor_pos_offset[2][0];
    }

    // Initialize decision values
    float low_distance = 2.4f / max_range;
    float leftStrong_value = 0.0f;
    float left_value = 0.0f;
    float forward_value = 0.0f;
    float right_value = 0.0f;
    float rightStrong_value = 0.0f;

    // Process sensor data for decision values
    for (uint8_t i = 0; i < 12; ++i) {
        float inputF = x[i];
        float foutput = 0.0f;

        //Gain function
        float lowGain=1.2f;
        float functionSlope=0.7f;
        float functionCorrection=1.3f;
        //*(-sqrt(powf(y[i]-lowGain*low_distance, 2.0f))/functionSlope -y[i]/functionSlope +functionCorrection);

        // Strong Left
        float leftDistance = 1.5f;
        float functionAngleRising = 0.75f;
        float functionCut = 0.75f;
        foutput = ((sqrtf(powf(inputF + leftDistance, 2.0f)) / functionAngleRising - inputF / functionAngleRising - leftDistance / functionAngleRising) -
                  (sqrtf(powf(inputF + leftDistance + functionCut, 2.0f)) / functionAngleRising - inputF / functionAngleRising - (leftDistance + functionCut) / functionAngleRising));
        leftStrong_value += y[i] * foutput*(-sqrt(powf(y[i]-lowGain*low_distance, 2.0f))/functionSlope -y[i]/functionSlope +functionCorrection);

        // Strong Left Secondary
        /*
        functionAngleRising = 2.0f;
        float functionLevel = 0.5f;
        float functionStart = 0.5f;
        float functionReduce = 4.0f;
        foutput = -(sqrtf(powf(-inputF + functionStart, 2.0f)) / functionAngleRising - (-inputF - functionStart) / functionAngleRising - functionLevel -
                   (sqrtf(powf(-inputF + 2.0f, 2.0f)) / functionAngleRising - (-inputF - 2.0f) / functionAngleRising - 2.0f)) / functionReduce;
        leftStrong_value += y[i] * foutput;
        */

        // Left
        inputF = x[i];
        foutput = 0.0f;
        functionAngleRising = 2.0f;
        float functionLevel = 0.5f;
        float functionStart = 0.5f;
        foutput = sqrtf(powf(inputF + functionStart, 2.0f)) / functionAngleRising - (inputF - functionStart) / functionAngleRising - functionLevel -
                  (sqrtf(powf(inputF + 2.0f, 2.0f)) / functionAngleRising - (inputF - 2.0f) / functionAngleRising - 2.0f);
        left_value += y[i] * foutput*(-sqrt(powf(y[i]-lowGain*low_distance, 2.0f))/functionSlope -y[i]/functionSlope +functionCorrection);

        // Middle
        inputF = x[i];
        foutput = 0.0f;
        foutput = (-sqrtf(powf(inputF - 0.5f, 2.0f)) / 2.0f - inputF / 2.0f + 2.0f) + (-sqrtf(powf(inputF + 0.5f, 2.0f)) / 2.0f + inputF / 2.0f + 0.5f);
        if (foutput < 0.0f) {
            foutput = 0.0f;
        }
        if ((y[i] - low_distance) < 0.0f) {
            forward_value += (y[i] - low_distance) * -foutput*(-sqrt(powf(y[i]-lowGain*low_distance, 2.0f))/functionSlope -y[i]/functionSlope +functionCorrection);
        }

        // Right
        inputF = x[i];
        foutput = 0.0f;
        functionAngleRising = 2.0f;
        functionLevel = 0.5f;
        functionStart = 0.5f;
        foutput = sqrtf(powf(-inputF + functionStart, 2.0f)) / functionAngleRising - (-inputF - functionStart) / functionAngleRising - functionLevel -
                  (sqrtf(powf(-inputF + 2.0f, 2.0f)) / functionAngleRising - (-inputF - 2.0f) / functionAngleRising - 2.0f);
        right_value += y[i] * foutput*(-sqrt(powf(y[i]-lowGain*low_distance, 2.0f))/functionSlope -y[i]/functionSlope +functionCorrection);

        // Strong Right
        inputF = x[i];
        foutput = 0.0f;
        foutput = ((sqrtf(powf(-inputF + leftDistance, 2.0f)) / functionAngleRising + inputF / functionAngleRising - leftDistance / functionAngleRising) -
                  (sqrtf(powf(-inputF + leftDistance + functionCut, 2.0f)) / functionAngleRising + inputF / functionAngleRising - (leftDistance + functionCut) / functionAngleRising));
        rightStrong_value += y[i] * foutput*(-sqrt(powf(y[i]-lowGain*low_distance, 2.0f))/functionSlope -y[i]/functionSlope +functionCorrection);

        // Strong Right Secondary
        /*
        functionAngleRising = 2.0f;
        functionLevel = 0.5f;
        functionStart = 0.5f;
        functionReduce = 4.0f;
        foutput = -(sqrtf(powf(inputF + functionStart, 2.0f)) / functionAngleRising - (inputF - functionStart) / functionAngleRising - functionLevel -
                   (sqrtf(powf(inputF + 2.0f, 2.0f)) / functionAngleRising - (inputF - 2.0f) / functionAngleRising - 2.0f)) / functionReduce;
        rightStrong_value += y[i] * foutput;
        */
    }

    // Determine the decision
    float command_values[] = {leftStrong_value, left_value, forward_value, right_value, rightStrong_value};
    float decisionValue = 0.0f;
    uint8_t decision = 0;
    uint8_t defaultCom = 0;

    for (uint8_t i = 0; i < 5; ++i) {
        if (command_values[i] > decisionValue && command_values[i] > decisionSensitivity) {
            decisionValue = command_values[i];
            decision = i + 1;
        } else {
            defaultCom++;
        }
        if (command_values(2)< (command_values(4)+0.025f) && command_values(2)> (command_values(4)-0.025f) && command_values(2)>command_values(3)){
        	defaultCom++;
        }
    }

    if (defaultCom >= 5) {
        decision = 0;
    }

    return decision;
}



