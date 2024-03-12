import can
import struct
import time

node_id = 1
channel = "/dev/tty.usbmodem1101"
bitrate = 500000

bus = can.interface.Bus(channel=channel, bustype="slcan", bitrate=bitrate)

# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

print("Step 1 complete")



    # Set velocity to 1.0 turns/s
bus.send(can.Message(arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
                    data=struct.pack('<ff', 1.0, 0.1), # 1.0: velocity, 0.0: torque feedforward
                    is_extended_id=False
                    ))

time.delay(1000)

    # Set velocity to 1.0 turns/s
bus.send(can.Message(arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
                    data=struct.pack('<ff', -1.0, 0.1), # 1.0: velocity, 0.0: torque feedforward
                    is_extended_id=False
                    ))

time.delay(1000)


# Construct the CAN message
message = can.Message(arbitration_id=(node_id << 5 | 0x02),
                        data=[0, 0, 0, 0, 0, 0, 0, 0],
                        is_extended_id=False)

# Print encoder feedback
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")

bus.shutdown()