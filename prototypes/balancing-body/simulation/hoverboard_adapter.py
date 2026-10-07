"""Exact repo serial codec and explicit torque calibration boundary. Opens no ports."""
import struct,math
import json,pathlib
MAX_TORQUE=json.loads((pathlib.Path(__file__).resolve().parents[1]/'parameters.json').read_text())['torque_limit_Nm_each']
START=0xABCD
def command(steer,speed):
 if not(-1000<=steer<=1000 and -1000<=speed<=1000):raise ValueError('source input range is +/-1000')
 return struct.pack('<HhhH',START,steer,speed,(START^(steer&65535)^(speed&65535))&65535)
def feedback(packet,*,odometry=False):
 length=22 if odometry else 18
 if len(packet)!=length:raise ValueError(f'source feedback length is {length} bytes for selected firmware mode')
 values=struct.unpack('<HhhhhhhhhHH' if odometry else '<HhhhhhhHH',packet);checksum=0
 for v in values[:-1]:checksum^=v&65535
 if values[0]!=START or values[-1]!=checksum:raise ValueError('feedback frame/checksum invalid')
 keys=['start','cmd1','cmd2','speedR_raw','speedL_raw']
 if odometry:keys+=['wheelR_count','wheelL_count']
 return dict(zip(keys+['battery_raw','board_temperature_raw','led_raw','checksum'],values))
def tank_torque_packet(left_Nm,right_Nm,*,left_Nm_per_unit=None,right_Nm_per_unit=None,left_polarity=None,right_polarity=None):
 """Requires TANK_STEERING and verified wheel signs after firmware's direction macros.
 No default converts amperes to Nm or labels an uncalibrated command as torque.
 """
 if left_Nm_per_unit is None or right_Nm_per_unit is None or left_polarity not in [-1,1] or right_polarity not in [-1,1]:raise ValueError('measured torque calibration and both wheel signs are required')
 if not all(math.isfinite(v) and v>0 for v in [left_Nm_per_unit,right_Nm_per_unit]):raise ValueError('invalid calibration')
 if not all(math.isfinite(v) and abs(v)<=MAX_TORQUE for v in [left_Nm,right_Nm]):raise ValueError('torque outside modelled envelope')
 left=round(left_polarity*left_Nm/left_Nm_per_unit);right=round(right_polarity*right_Nm/right_Nm_per_unit)
 # Firmware main.c: TANK_STEERING cmdL=steer, cmdR=speed.
 return command(left,right)
