"""Fetch external baselines into sibling directories; never change another checkout."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parent
WORK=ROOT.parents[2]
PINS=[('source-soarm','https://github.com/egc365/SO-ARM100.git','5f6d2b876a53a4872e405b991dd925556c9e38a4'),
      ('source-lekiwi','https://github.com/SIGRobotics-UIUC/LeKiwi.git','efa608d7ee5a495a4803b1d28cd0c955b4f1e033')]
for folder,url,sha in PINS:
    p=WORK/folder
    if not p.exists():
        p.mkdir();subprocess.run(['git','init',str(p)],check=True)
        subprocess.run(['git','-C',str(p),'fetch','--depth','1',url,sha],check=True)
        subprocess.run(['git','-C',str(p),'checkout','--detach','FETCH_HEAD'],check=True)
    actual=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
    if actual!=sha:raise RuntimeError(f'{folder} is at {actual}; expected {sha}. Existing checkout was not changed.')
    print(folder,sha)
