import numpy as np
import matplotlib.pyplot as plt


class Bezier:
    def __init__(self, control_points: list[np.ndarray]):
        if len(control_points) != 4:
            raise ValueError("Only supports 4 control points")
        self.control_points = control_points
    
    def evaluate(self, t: float | np.ndarray) -> np.ndarray:
        if isinstance(t, np.ndarray):
            t = t[:, np.newaxis]
        return self.control_points[0] * (1 - t) ** 3 + 3 * self.control_points[1] * t * (1 - t) ** 2 + 3 * self.control_points[2] * t ** 2 * (1 - t) + self.control_points[3] * t ** 3
    
    def derivative(self, t: float) -> np.ndarray:
        return 3 * (self.control_points[1] - self.control_points[0]) * (1 - t) ** 2 + 6 * (self.control_points[2] - self.control_points[1]) * t * (1 - t) + 3 * (self.control_points[3] - self.control_points[2]) * t ** 2
    
    def derivature_2(self, t: float) -> np.ndarray:
        return 6 * (self.control_points[2] - 2 * self.control_points[1]) * (1 - t) + 6 * (self.control_points[3] - self.control_points[2]) * t
    
    def curvature(self, t: float) -> float:
        return np.linalg.norm(self.derivative(t)) / np.linalg.norm(self.derivature_2(t))

    def plot(self, ax, N: int = 100) -> None:
        t = np.linspace(0, 1, N)
        points = np.array([self.evaluate(t) for t in t])
        ax.plot(points[:, 0], points[:, 1])
        ax.set_aspect("equal")

def analytical_solution(start: np.ndarray, start_yaw: float, end: np.ndarray, end_yaw: float) -> Bezier:
    delta = end - start
    t0 = np.array([np.cos(start_yaw), np.sin(start_yaw)])
    t3 = np.array([np.cos(end_yaw), np.sin(end_yaw)])
    delta_dot_t0 = np.dot(delta, t0)
    delta_dot_t3 = np.dot(delta, t3)
    c = np.dot(t0, t3)
    denom = 4 * (4 - c * c)
    alpha = 3 * (2 * delta_dot_t0 + c * delta_dot_t3) / denom
    beta  = 3 * (2 * delta_dot_t3 + c * delta_dot_t0) / denom
    # denom = 25 - 16 * c * c
    # alpha = 3 * (5 * delta_dot_t0 - 4 * c * delta_dot_t3) / denom
    # beta  = 3 * (5 * delta_dot_t3 - 4 * c * delta_dot_t0) / denom
    print("alpha:", alpha)
    print("beta:", beta)
    P1 = start + alpha * t0
    P2 = end - beta * t3
    return Bezier([start, P1, P2, end])

def symmetric_solution(start: np.ndarray, start_yaw: float, end: np.ndarray, end_yaw: float, weight: float) -> Bezier:
    delta = end - start
    t0 = np.array([np.cos(start_yaw), np.sin(start_yaw)])
    t3 = np.array([np.cos(end_yaw), np.sin(end_yaw)])
    c = np.dot(t0, t3)
    D = np.dot(delta, t0) + np.dot(delta, t3)
    numerator = (1 + 12 * weight) * D
    denominator = 2 * (2 + c) + 36 * weight * (1 + c)
    gamma = numerator / denominator
    print("gamma:", gamma)
    P1 = start + gamma * t0
    P2 = end - gamma * t3
    return Bezier([start, P1, P2, end])

def main():
    start = np.array([0, 0])
    start_yaw = -np.pi / 2
    end = np.array([2, 1])
    end_yaw = 0

    fig, ax = plt.subplots()
    for weight in np.linspace(0, 0.5, 1):
        bezier = symmetric_solution(start, start_yaw, end, end_yaw, weight)
        bezier.plot(ax)
        ts = np.array([0, 0.1, 0.2, 0.3])
        print(bezier.evaluate(ts))
    
    plt.show()

if __name__ == "__main__":
    main()
