## Prerequisites
You need access to the incar packages and the following packages installed

```bash
pip install incar_networking
pip install Robotic_Arm
```

If you want to additionally use the inspire hand, make sure that the inspire extension is also installed:
```bash
git clone https://github.com/INCAR-Robotics/inspire_extension.git
cd inspire_extension
pip install .
```

## Run
```bash
python3 run.py
```
Or to run along with the inspire hand
```bash
python3 run_with_inspire_hand.py
```