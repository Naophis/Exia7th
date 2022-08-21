import numpy as np
import matplotlib.pyplot as plt

from slalom import Slalom
from plot import Plot

p = Plot()

v = 800
show = True

# p.exe("normal", v,show)
p.exe("large", v, show)
# p.exe("orval", v, show)  # not use
p.exe("dia45", v, show)
p.exe("dia45_2", v, show)
p.exe("dia135", v, show)
p.exe("dia135_2", v, show)
p.exe("dia90", v, show)
