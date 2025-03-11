# How to Add Messages to CAVeTalk

Adding new messages to the CAVeTalk protocol is relatively simple, but guides are nice.

## 1. Update README & Create `.proto` file & Update `ids.proto`

README - Add these items to the preexisting table
- ID
- Name
- Description

---

`.proto` file - template file for Protocol Buffers Packet Information

Use the other files as reference but here are the minimum requirements:
- `syntax = "proto3";`
- `package cave_talk;`
- Message Definition Example:
    ```
    message Movement {
        double speed_meters_per_second = 1;
        double turn_rate_radians_per_second = 2;
    }
    ```
- Submessage Definition
    ```
    message Servo {
        double min_angle_radian = 1;
        double max_angle_radian = 2;
        double center_angle_radian = 3;
        double min_duty_cycle_microseconds = 4;
        double max_duty_cycle_microseconds = 5;
        double center_duty_cycle_microseconds = 6;
    }

    message ConfigServoWheels {
        Servo servo_wheel_0 = 1;
        Servo servo_wheel_1 = 2;
        Servo servo_wheel_2 = 3;
        Servo servo_wheel_3 = 4;
    }
    ```

    - Make a note of the ascending field numbers for each field
    - They must be unique otherwise protobufs gets fussy
---

#### Update `ids.proto` to include the new message name and field
## 2. Update Lib Program Files

Update the Source and Include files in `lib/c` & `lib/c++`

It'd be best to initialize the new protobuf message using the build directions in the README. **Build both C & C++**

`output params` refers to the fields defined in the top level of your message. That might be built-ins or submessage types depending on the message you are trying to send.

### C
---
- Try to pass struct parameters as pointers
- Try to pass all parameters as const
- Try to pass any pointer params as *const


#### `cave_talk.h`
---

- Add a function pointer callback to `CaveTalk_ListenCallbacks_t`
    - Output: `void`
    - Input: `output params`
- Add a `CaveTalk_Speak______` function
    - Output: `CaveTalk_Error_t`
    - Input: `"handle" ptr`, `output params`

#### `cave_talk.c`
---

- Include the new message's `.pb.h` file
- Add static `CaveTalk_Handle_______` function
    - Output: `CaveTalk_Error_t`
    - Input: `"handle" ptr`, `length`
- Add Message ID and Handle Function to Switch Case in `CaveTalk_Hear`
- Write the definition of the `CaveTalk_Speak______` function
    - Follow the format of the other functions
- Write static `CaveTalk_Handle_______` function implementation


### C++
---
- Try to pass in non-built-in objects as references
- Try to pass all parameters as const

#### `cave_talk.h`
---

- Add ListenerCallback virtual function for message
    - Output: `void`
    - Input: `output params`
- Add `Handle______` function to `Listener` class
    - Output: `CaveTalk_Error_t`
    - Input: `length`
- Add `Speak______` function to `Talker` class
    - Output: `CaveTalk_Error_t`
    - Input: `output params`


#### `cave_talk.c`
---

- Include the new message's `.pb.h` file
- Add Message ID and Handle Function to Switch Case in `Listener::Listen`
- Write the definition of the `Handle______` function
    - Follow the format of the other functions
- Write the definition of the `Speak______` function
    - Follow the format of the other functions


## 3. Update Unit Tests

Create `C` & `C++` unit tests for the new messages using the other messages as a guide for the new message's tests. 

Good tests have these qualities:
- Thorough
- Succinct
- Matches real use
- Tests foreseeable edge cases
- Tests odd input values
- Utilizes Mock functions where possible
    - If values cannot be checked directly through tests, leave a comment where a breakpoint could be used to see values


