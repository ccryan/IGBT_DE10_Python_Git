hall_reading	read one address sensor reading for 20 times 
igbt_pulser	able to pulse 18 channels freely, and read one address sensor for 5 times

igbt_pulser_for_magnet_control 
igbt_pulser_backup_20200624: only 18 channels pulse, no sensor readings
20200624: increase to 36 channels

hallArray_reading
./hallArray_reading
hall array reading for 2 times. 

hall_reading_multi
./hall_reading_multi sensor1 sensor2 .. sensor9 num_iter enable_mesage
read 9 address, define number of iterations, define enable message

hall_reading_multi_ver2
./hall_reading_multi sensor1 sensor2 .. sensor9 num_iter enable_mesage
speed up the hall reading 

hallArray_readingSingleBoard 
./hallArray_readingSingleBoard NoIteration Enable_Message
Use i2c_int channel, define number of iterations, define enable message