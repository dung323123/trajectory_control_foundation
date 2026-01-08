import math
import typing as T

import numpy as np
from numpy import linalg
from scipy.integrate import cumulative_trapezoid
import matplotlib.pyplot as plt  # type: ignore

from utils import save_dict, maybe_makedirs

class State:
    def __init__(self, x: float, y: float, V: float, th: float) -> None:
        self.x = x
        self.y = y
        self.V = V
        self.th = th

    @property
    def xd(self) -> float:
        return self.V*np.cos(self.th)

    @property
    def yd(self) -> float:
        return self.V*np.sin(self.th)


def compute_traj_coeffs(initial_state: State, final_state: State, tf: float) -> np.ndarray:
    """
    Inputs:
        initial_state (State)
        final_state (State)
        tf (float) final time
    Output:
        coeffs (np.array shape [8]), coefficients on the basis functions

    Hint: Use the np.linalg.solve function.
    """
    x0, y0 = initial_state.x, initial_state.y
    xf, yf = final_state.x, final_state.y

    xd0 = initial_state.V * np.cos(initial_state.th)
    yd0 = initial_state.V * np.sin(initial_state.th)

    xdf = final_state.V * np.cos(final_state.th)
    ydf = final_state.V * np.sin(final_state.th)

    A = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*tf**2]
    ])

    bx = np.array([x0, xd0, xf, xdf])
    by = np.array([y0, yd0, yf, ydf])

    ax = np.linalg.solve(A, bx)
    ay = np.linalg.solve(A, by)

    coeffs = np.hstack((ax, ay))
    return coeffs
    return coeffs

def compute_traj(coeffs: np.ndarray, tf: float, N: int) -> T.Tuple[np.ndarray, np.ndarray]:
    """
    Inputs:
        coeffs (np.array shape [8]), coefficients on the basis functions
        tf (float) final_time
        N (int) number of points
    Output:
        t (np.array shape [N]) evenly spaced time points from 0 to tf
        traj (np.array shape [N,7]), N points along the trajectory, from t=0
            to t=tf, evenly spaced in time
    """
    t = np.linspace(0, tf, N)
    traj = np.zeros((N, 7))

    ax = coeffs[0:4]
    ay = coeffs[4:8]

    for i, ti in enumerate(t):
        x = ax[0] + ax[1]*ti + ax[2]*ti**2 + ax[3]*ti**3
        y = ay[0] + ay[1]*ti + ay[2]*ti**2 + ay[3]*ti**3

        xd = ax[1] + 2*ax[2]*ti + 3*ax[3]*ti**2
        yd = ay[1] + 2*ay[2]*ti + 3*ay[3]*ti**2

        xdd = 2*ax[2] + 6*ax[3]*ti
        ydd = 2*ay[2] + 6*ay[3]*ti

        th = np.arctan2(yd, xd)

        traj[i, :] = [x, y, th, xd, yd, xdd, ydd]

    return t, traj

def compute_controls(traj: np.ndarray) -> T.Tuple[np.ndarray, np.ndarray]:
    """
    Input:
        traj (np.array shape [N,7])
    Outputs:
        V (np.array shape [N]) V at each point of traj
        om (np.array shape [N]) om at each point of traj
    """
    xd = traj[:,3]
    yd = traj[:,4]
    xdd = traj[:,5]
    ydd = traj[:,6]
    th = traj[:,2]

    V = np.sqrt(xd**2 + yd**2)
    V[V < 1e-3] = 1e-3  # tránh singularity

    om = (-np.sin(th)*xdd + np.cos(th)*ydd) / V

    return V, om

if __name__ == "__main__":
    # Constants
    tf = 25.

    # time
    dt = 0.005
    N = int(tf/dt)+1
    t = dt*np.array(range(N))

    # Initial conditions
    s_0 = State(x=0, y=0, V=0.5, th=-np.pi/2)

    # Final conditions
    s_f = State(x=5, y=5, V=0.5, th=-np.pi/2)

    coeffs = compute_traj_coeffs(initial_state=s_0, final_state=s_f, tf=tf)
    t, traj = compute_traj(coeffs=coeffs, tf=tf, N=N)
    V,om = compute_controls(traj=traj)

    maybe_makedirs('plots')

    # Plots
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.plot(traj[:,0], traj[:,1], 'k-',linewidth=2)
    plt.grid(True)
    plt.plot(s_0.x, s_0.y, 'go', markerfacecolor='green', markersize=15)
    plt.plot(s_f.x, s_f.y, 'ro', markerfacecolor='red', markersize=15)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title("Path (position)")
    plt.axis([-1, 6, -1, 6])

    ax = plt.subplot(1, 2, 2)
    plt.plot(t, V, linewidth=2)
    plt.plot(t, om, linewidth=2)
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.legend(['V [m/s]', '$\omega$ [rad/s]'], loc="best")
    plt.title('Original Control Input')
    plt.tight_layout()

    plt.savefig("plots/differential_flatness.png")
    plt.show()
