import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lv/Desktop/Git/metr4202_2024_team20/search_algorithm/install/autopilot_pkg'
