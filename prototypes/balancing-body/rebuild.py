"""Reproduce CAD, simulation evidence and review visuals. Does not energize hardware."""
import argparse, pathlib, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('--skip-render',action='store_true')
args=parser.parse_args()
steps=['cad/build.py','simulation/build_model.py','simulation/balance.py',
       'simulation/firmware_profile.py','checks/engineering.py','checks/software.py',
       'checks/interactions.py','simulation/contact_check.py','checks/parts.py',
       'checks/review_plan.py','visuals/charts.py','visuals/sections.py']
if not args.skip_render:steps.append('visuals/render.py')
steps.append('review_package.py')
for step in steps:
 print(f'Rebuilding {step}',flush=True)
 subprocess.run([sys.executable,str(ROOT/step)],cwd=ROOT,check=True)
print('CAD and review rebuilt. Recorded simulation failures and physical blockers remain; read README.md.')
