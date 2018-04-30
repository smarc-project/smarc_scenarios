# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np, matplotlib.pyplot as plt

class Algae_Farm(object):

    def __init__(self, dx, dy, dw, lx, N, dsx, dsy):

        # x and y padding between wall area and farm borders [m]
        self.dx, self.dy = dx, dy
        # seperation between walls [m]
        self.dw = dw
        # length of walls and x length of wall space [m]
        self.lx = lx
        # number of walls
        self.N = N
        # docking station position [m]
        self.dsx, self.dsy = dsx, dsy

        # y length of wall space
        self.ly = (self.N - 1)*self.dw
        # x and y length of farm borders
        self.Lx, self.Ly = self.lx + 2*self.dx, self.ly + 2*self.dy
        # area of wall space
        self.Aw = self.lx*self.ly
        # area of farm space
        self.Af = self.Lx*self.Ly
        # wall indicies
        self.wi = range(self.N)

    def valid_wall(self, n):

        # make sure wall index is valid
        if n in self.wi:
            return True
        else:
            raise ValueError("Not a valid wall index.")

    def wall(self, n):

        # return xy coordinates of wall-n start and end
        if self.valid_wall(n):
            return self.dx, self.dy + self.dw*n, self.dx + self.lx, self.dy + self.dw*n

    def wall_pts(self, n, npts=100):

        # start and end of wall
        x0, y0, xf, yf = self.wall(n)
        # x and y points
        return np.linspace(x0, xf, npts), np.linspace(y0, yf, npts)

    def border(self, bi):

        # north wall
        if bi == 0:
            return 0, self.Ly, self.Lx, self.Ly
        # east wall
        elif bi == 1:
            return self.Lx, 0, self.Lx, self.Ly
        # south wall
        elif bi == 2:
            return 0, 0, self.Lx, 0
        # west wall
        elif bi == 3:
            return 0, 0, 0, self.Ly

    def border_pts(self, bi, npts=100):

        # start and end of border
        x0, y0, xf, yf = self.border(bi)
        # x and y points
        return np.linspace(x0, xf, npts), np.linspace(y0, yf, npts)

    def plot(self, ax=None):

        # create axis if not provided
        if ax is None:
            fig = plt.figure()
            ax = fig.gca()

        # plot farm borders
        for bi in range(4):
            x, y = self.border_pts(bi)
            ax.plot(x, y, "k-", linewidth=5)

        # plot algae walls
        for wi in self.wi:
            x, y = self.wall_pts(wi)
            ax.plot(x, y, "k-")

        # plot docking station
        ax.scatter(self.dsx, self.dsy, s=80, c="k", marker="x")

        # set limits
        #ax.set_xlim([0, self.Lx])
        #ax.set_ylim([0, self.Ly])

        return ax

if __name__ == "__main__":

    # initialise farm
    farm = Algae_Farm(5, 5, 5, 10, 10, 40, 70)
    # plot farm layout
    farm.plot()
    # show plot
    plt.show()
