1. 1_saturate_right_positive

The script apply 20 positive pulses with 50 mks duration to right magnet head. 
Back Hall sensors are not red in between the pulse.
Only after all pulses are accomplished the script reads the back Hall sensors.
The Hall sensor readings are saved into a file in HallSensorData folder.
25 readings from each sensor are performed.

2. 2_Apply_1NegativePulse_rightColumn_RightHead

The script applies 1 negative pulse to three magnets in the right head (21, 24, 27). 
These three magnets forms a column at the right side of the head (loking from negative z direction in lab frame).
The pulse duration can be changes in the script (u variable).
Only after the pulse is accomplished the script reads the back Hall sensors.
The Hall sensor readings are saved into a file in HallSensorData folder.
25 readings from each sensor are performed.

3. 3_Read_Right_HS

Read all 9 Hall sensors of the right head.
25 readings are performed.
The Hall sensor readings are saved into a file in HallSensorData folder.

4. 4_saturate_left_negative

The script apply 20 negative pulses with 50 mks duration to left magnet head. 
Back Hall sensors are not red in between the pulse.
Only after all pulses are accomplished the script reads the back Hall sensors.
The Hall sensor readings are saved into a file in HallSensorData folder.
25 readings from each sensor are performed.

5. 5_Apply_1PositivePulse_RightColumn_LeftHead

The script applies 1 positive pulse to three magnets in the left head (1, 4, 7). 
These three magnets forms a column at the right side of the head (loking from negative z direction in lab frame).
The pulse duration can be changes in the script (u variable).
Only after the pulse is accomplished the script reads the back Hall sensors.
The Hall sensor readings are saved into a file in HallSensorData folder.
25 readings from each sensor are performed.

3. 3_Read_Left_HS

Read all 9 Hall sensors of the left head.
25 readings are performed.
The Hall sensor readings are saved into a file in HallSensorData folder.