import numpy as np
import matplotlib.pyplot as plt

from slalom import Slalom
from plot import Plot
from plotorval import PlotOrval

p = Plot()
po = PlotOrval()

v = 500
# show = True
show = False


# p.exe("normal", v,show ,0)
# p.exe("large", v, show ,0)
# p.exe("orval", v, show ,0)  
p.exe("dia45", v, show, 0)
# p.exe("dia45", v, show, 1)
# p.exe("dia45_2", v, show ,0)
# p.exe("dia135", v, show ,0)
# p.exe("dia135_2", v, show ,0)
# p.exe("dia90", v, show ,0)
