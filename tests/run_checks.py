"""Run repository host checks; no robot connection or motor commands."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent.parent
with tempfile.TemporaryDirectory() as directory:
    for source in sorted((root/'tests').glob('test_*.cpp')):
        binary = str(Path(directory)/source.stem)
        subprocess.run(['c++','-std=c++14','-Wall','-Wextra','-Werror','-I',str(root/'tests'),str(source),'-o',binary],check=True,cwd=root)
        subprocess.run([binary],check=True,cwd=root)
        print('PASS', source.name, flush=True)
subprocess.run(['python3','-m','unittest','discover','-s','controller','-p','test*.py'],check=True,cwd=root)
subprocess.run(['node','controller/test_app.js'],check=True,cwd=root)
