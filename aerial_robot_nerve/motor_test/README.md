## Setting
1. Very important: set `NERVE_COMM = 0` and write firmware to spinal.

2. Connect like this:

 ``````````````````````````````````
 ``````````````````````````````````
 ```  PC --spinal--ESC---rotor  ```
 ```  |              |          ```
 ```  |-- power supply          ```
 ```  |                         ```
 ```  |--force sensor           ```
 ``````````````````````````````````
 ``````````````````````````````````
3. Create new wired setting for power supply. Set IP address of the power supply to `192.168.0.2`, Netmask to '255.255.255.0', and Gateway to 192.168.0.1.
4. Power on the power supply and set the voltage as you like (e.g. 25.2V).
5. In your terminal, do:
   ```
   $ roslaunch motor_test test.launch
   ```

   **parameter**:
   - `test_mode`: the test mode. Step mode (:=0) is to give continous steps command to the ESC, the problem is the bias of the force sensor increases during the steps commands. Thus, we recommand to use **one-shot** mode (:=1, default mode), which iteratively gives an increasing PWM value. The difference is this mode stops the motor rotation after each step, and re-calibrate the force sensor before the next step. So the bias of force sensor can be considered to be zero.
   - `run_duration`: the duration of motor rotation during each step.
   - `pwm_incremental_value`: the increamental value between two steps.
   - `min_pwm_value`/`max_pwm_value`: the min/max of the pwm value
   - `motor_index`: the motor output index to test. The default is 0. For stm32H7_v2, 0: PE9/TIM1_CH1, 1: PE11/TIM1_CH2, 2: PE13/TIM1_CH3, 3: PE14/TIM1_CH4, 4: PD12/TIM4_CH1, 5: PD13/TIM4_CH2, 6: PD14/TIM4_CH3, 7: PD15/TIM4_CH4.
   - `raise_duration`/`brake_duration`: these are the special parameters for one-shot mode, since we have to consider the raise-up phase after staring rotation and brake-down phase after stopping rotation. For small propellers (e.g. 5inch), these value can be small. For large propellers (e.g. 14inch), please increase there values
   - `force_sensor`: the type of force sensor. Options are CFS034CA301U (default); PFS055YA501U6 (larger type)

   **sample**:
   - for 5inch propeller:
     ```
     $ roslaunch motor_test test.launch max_pwm_value:=1900 raise_duration:=1.0 brake_duration:=2.0 run_duration:=2.0
     ```

   - for testing only the first output pin:
     ```
     $ roslaunch motor_test test.launch motor_index:=0
     ```

   - for 5inch propeller:
     ```
     $ roslaunch motor_test test.launch max_pwm_value:=1200 raise_duration:=2  brake_duration:=5 run_duration:=2
     ```

6. Start logging by `rostopic pub -1 /start_log_cmd std_msgs/Empty "{}" `.

7. Cool down your motor by sending `rostopic pub -1 /pwm_test spinal/PwmTest "{motor_index: [0], pwms: [0.54]}"` and when it is cool enough, send `rostopic pub -1 /pwm_test spinal/PwmTest "{motor_index: [0], pwms: [0.5]}"`, and then start a next test. Replace `0` with the `motor_index` used for the test.

## General calib for ESC:

Low PWM: 1000 (start with 1050)
High PWM: 1900
