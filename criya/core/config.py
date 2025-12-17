CONFIG_FILE = "config.json"

DEFAULT_CONFIG = {
    "port": "",
    "baud": 9600,
    "speed": 50,
    "geometry": {
        "L_BASE": 120,
        "L_HUMERUS": 120,
        "L_SEG1": 70,
        "L_SEG2": 85,
        "L_SEG3": 75,
        "L_TOOL": 120
    },
    "calibration": [90, 130, 110, 100, 95, 95, 90],
    "positions": {},   # Name -> [angles]
    "sequence": []     # list of steps
}
