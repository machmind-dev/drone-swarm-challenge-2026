import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pihas/drone-swarm-challenge-2026/ground-station-software/gcs/rqt/install/machmind_rqt_buttons'
