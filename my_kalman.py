import numpy as np
import matplotlib.pyplot as plt

def main():
    t_end = 100
    num_samples = 10000
    t = np.linspace(0, t_end, num_samples)
    dt = (t_end) / (num_samples - 1)

    F = np.array([[1, dt],[0, 1]])

    H = np.array([1, 0])
    R = 0.01
    Q = np.array([[1, 0], [0, 1]])
    w = 0.1
    true_x = np.sin(w*t) # quadratic position
    true_x_dot = w * np.cos(w*t) # linear speed

    x_hat = np.array([[true_x[0]],[true_x_dot[0]]])
    P = np.array([[0.5, 0],[0, 0.5]])
    x_hats = [true_x[0]]
    x_dot_hats = [true_x_dot[0]]

    for i in range(1,len(t)):
        # predict current based on previous
        x_hat_predict = F @ x_hat
        P = F @ P @ F.T + Q

        # update according to measurement

        z = true_x[i] + np.random.normal(0, R)
        y = z - H @ x_hat_predict

        S = H @ P @ np.atleast_2d(H).T + R

        K = P @ np.atleast_2d(H).T / S

        x_hat = x_hat_predict + K @ np.atleast_2d(y)
        P = (np.identity(2) - K @ np.atleast_2d(H)) @ P
        x_hats.append(x_hat[0][0])
        x_dot_hats.append(x_hat[1][0])


    plt.rcParams['text.usetex'] = True
    fig, ax = plt.subplots(2,1)
    plt.subplot(2,1,1)
    plt.plot(t, x_hats, label = r"estimated state")
    plt.plot(t, true_x, label = r"true state")
    plt.xlabel(r"Time [s]")
    plt.ylabel(r"$x$ [m]")
    plt.legend()

    plt.subplot(2,1,2)
    plt.plot(t, x_dot_hats, label = r"estimated state")
    plt.plot(t, true_x_dot, label = r"true state")
    plt.xlabel(r"Time [s]")
    plt.ylabel(r"$\dot{x}$ [m/s]")
    plt.legend()
    plt.show()
    fig.savefig('myimage.svg', format='svg', dpi=1200)

    



if __name__ == "__main__":
    main()