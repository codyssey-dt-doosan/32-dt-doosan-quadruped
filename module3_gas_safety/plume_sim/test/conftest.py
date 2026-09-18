# 소스 트리에서 바로 pytest를 돌릴 때 옆 패키지(plume_sim, return_to_home)를 찾게 한다
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for pkg in ("plume_sim", "return_to_home", "source_seeking"):
    path = os.path.normpath(os.path.join(HERE, "..", "..", pkg))
    if path not in sys.path:
        sys.path.insert(0, path)
