from utilities import plot_quad
from numpy import arange
def update_plot_Quad(ax,Quad):
    for i in arange(start=0,stop=len(Quad),step=1):
        Quad[i].armX.remove()
        Quad[i].armY.remove()
        Quad[i].motorX1[0].remove()
        Quad[i].motorX2[0].remove()
        Quad[i].motorY1[0].remove()
        Quad[i].motorY2[0].remove()

    Quad = plot_quad.plot_quad(ax, Quad)

