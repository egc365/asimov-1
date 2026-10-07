"""Generate reviewable changes to pinned firmware; no flashing or build claim."""
import pathlib,difflib,json
ROOT=pathlib.Path(__file__).resolve().parents[1]
src=(ROOT/'source/hoverboard_config.h').read_text();modified=src
# Exact source spacing may differ; target the unique macro token with anchored regex.
import re
def macro(text,name,value):
 pattern=r'(^#define\s+'+name+r'\s+)\S+'
 result,n=re.subn(pattern,lambda m:m.group(1)+value,text,flags=re.M)
 if n!=1:raise ValueError('expected unique source macro '+name)
 return result
for name,value in [('CTRL_MOD_REQ','TRQ_MODE'),('DEFAULT_RATE','32767'),('DEFAULT_FILTER','65535')]:modified=macro(modified,name,value)
modified,n=re.subn(r'^// #define ENABLE_ODOMETRY', '#define ENABLE_ODOMETRY',modified,flags=re.M);assert n==1
modified,n=re.subn(r'^  // #define TANK_STEERING              // use for tank steering, each input controls each wheel','  #define TANK_STEERING                 // independent commands: steer=left, speed=right',modified,flags=re.M);assert n==1
main=(ROOT/'source/hoverboard_main.c').read_text();main2,n=re.subn(r'if \(main_loop_counter % 2 == 0\)',r'if (main_loop_counter % 1 == 0)',main);assert n==1
main2=main2.replace('if (main_loop_counter % 1 == 0) {    // Send data periodically every 10 ms','if (main_loop_counter % 1 == 0) {    // Candidate balance telemetry: each iteration; measure timing')
patch=''.join(difflib.unified_diff(src.splitlines(True),modified.splitlines(True),fromfile='a/Inc/config.h',tofile='b/Inc/config.h'))
patch+=''.join(difflib.unified_diff(main.splitlines(True),main2.splitlines(True),fromfile='a/Src/main.c',tofile='b/Src/main.c'))
(ROOT/'simulation/hoverboard_review.patch').write_text(patch)
print(json.dumps({'patch':'hoverboard_review.patch','changed_macros':5,'feedback':'each main-loop iteration; ENABLE_ODOMETRY changes wire feedback to 22 bytes','not_implemented':'real-time IMU task, sensor fusion, board wiring, current-to-Nm calibration, closed-loop scheduling; this patch alone DOES NOT balance'}))
