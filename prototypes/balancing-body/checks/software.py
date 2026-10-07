"""C/Python controller equivalence and source protocol interoperability/fault checks."""
import pathlib,sys,json,ctypes,subprocess,struct,numpy as np,runpy,copy
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from balance import gain
from hoverboard_adapter import command,feedback,tank_torque_packet
libpath=ROOT/'checks/balance_core.so'
subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fPIC','-shared',str(ROOT/'simulation/balance_core.c'),'-lm','-o',str(libpath)],check=True)
class State(ctypes.Structure):_fields_=[(k,ctypes.c_double) for k in ['x','v','pitch','rate','roll','age','yaw']]
class Output(ctypes.Structure):_fields_=[('left',ctypes.c_double),('right',ctypes.c_double),('status',ctypes.c_int)]
lib=ctypes.CDLL(str(libpath));lib.balance_step.argtypes=[ctypes.POINTER(State)];lib.balance_step.restype=Output
K,*_=gain();rng=np.random.default_rng(7);maxerror=0
for _ in range(1000):
 s=rng.normal(size=4)*[.05,.05,.02,.03];st=State(*s,0,.005,0);out=lib.balance_step(ctypes.byref(st));expected=np.clip(-K@s/2,-6,6);assert out.status==0;maxerror=max(maxerror,abs(out.left-expected));assert abs(out.left-expected)<1e-10
for values,expected in [([0,0,float('nan'),0,0,.005,0],1),([0,0,0,0,0,.025,0],1),([0,0,.2,0,0,.005,0],2),([0,0,0,0,.1,.005,0],2)]:
 st=State(*values);out=lib.balance_step(ctypes.byref(st));assert out.status==expected
st=State(0,0,.08,0,0,.005,10);out=lib.balance_step(ctypes.byref(st));assert abs(out.left)<=6 and abs(out.right)<=6
# Independent known wire frame: START ABCD, steer -50=FFCE, speed 1000=03E8, XOR 57EB.
assert command(-50,1000).hex()=='cdabceffe803eb57'
buf=(ctypes.c_uint8*8)();lib.hoverboard_command(buf,-50,1000);assert bytes(buf)==command(-50,1000)
words=[0xABCD,-50,1000,120,118,3700,250,3];chk=0
for v in words:chk^=v&65535
packet=struct.pack('<HhhhhhhHH',*words,chk);assert feedback(packet)['speedL_raw']==118
odomwords=words[:5]+[-32768,32767]+words[5:];odomchk=0
for v in odomwords:odomchk^=v&65535
odompacket=struct.pack('<HhhhhhhhhHH',*odomwords,odomchk);assert feedback(odompacket,odometry=True)['wheelR_count']==-32768
try:feedback(odompacket);raise AssertionError('odometry mode mismatch accepted')
except ValueError:pass
for bad in [packet[:-1],packet[:-1]+bytes([packet[-1]^1])]:
 try:feedback(bad);raise AssertionError('bad feedback accepted')
 except ValueError:pass
try:tank_torque_packet(1,1);raise AssertionError('uncalibrated torque accepted')
except ValueError:pass
# Skill-required negative CAD parameter checks, invoking the editable native validator.
namespace=runpy.run_path(str(ROOT/'cad/build.py'));p=namespace['P'];validate=namespace['validate'];negative=0
for key,value in [('wheel_radius_mm',50),('column_wall_mm',0),('column_wall_mm',40),('shaft_bore_mm',14),('wheel_track_mm',300),('deck_thickness_mm',3)]:
 q=copy.deepcopy(p);q[key]=value
 try:validate(q);raise AssertionError(f'invalid {key} accepted')
 except ValueError:negative+=1
result={'C_compile':'passed -Wall -Wextra -Werror','C_Python_cases':1000,'max_torque_error_Nm':maxerror,'fault_cases':4,'yaw_saturation':'passed','source_serial_golden_frame':'cdabceffe803eb57','feedback_18_and_22_byte_modes':'passed, mode mismatch rejected','bad_feedback_cases':2,'uncalibrated_driver':'rejected','negative_CAD_parameters':negative,'hardware_port':'not built/flashed; IMU drivers, real-time scheduling and Nm/current calibration still required'}
(ROOT/'checks/software_results.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
