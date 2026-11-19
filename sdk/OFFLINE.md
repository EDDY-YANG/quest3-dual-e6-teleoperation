# Offline algorithms

Run the default software verification with the VS Code task **SDK: Verify native solver offline (Windows x64)**. It uses the public manufacturer's URDF flange geometry, tests the shipped native DLL and bounded DLS against a small mathematical target, and checks the local HTTP routes without constructing a robot driver. Online control still identifies and verifies its own model from the connected controller.

Package 11 contains the research implementation of RangedIK reference planning followed by a constrained DLS update. It uses data supplied by the person running the experiment. After SDK setup, install the research dependencies in the project environment only if you need replay:

~~~powershell
.\app\.venv\Scripts\python.exe -m pip install -r .\app\requirements-offline.txt
.\app\.venv\Scripts\python.exe .\app\experiments\ranged_dls\run_comparison.py --help
~~~

Pass `--model` (your SDK-generated model JSON with a `model` field), `--recordings` (the parent of your recording directories), `--episodes` (directory names with one eligible recorded Grip and events.jsonl), `--down-seed-deg` (six angles chosen for an offline comparison), and `--output` (a new directory). For the separate DLS study, supply `--model`, `--prepare`, `--episode` and `--output`. For the model verification tool `ranged_ik_offline_check.py`, supply your own `--fk-samples` with identification and held-out controller FK samples. The distribution includes no personal recording, stored FK query set, return target or original study output. Generated traces represent model references. The original hardware comparison and its selection criteria are available on the [project website](https://eddy-yang.github.io/quest3-dual-e6-teleoperation/analysis/methods.html).
