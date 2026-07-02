"""Extract consolidated numbers from the closed-loop RTB sim for the report."""
import numpy as np
from braco_rtb import ik, fk_position, gravload, robot
from closed_loop_sim import quintic_coeffs, quintic_eval, KP, KD
from extra_plots import simulate

TARGET = (-0.155, 0.290, 0.361)
T = 4.0
q0 = np.zeros(5)
q_end, ok, err = ik(TARGET)
print('IK ok:', ok, 'err_mm:', round(err*1000, 4))
print('q_end deg/mm:', [round(np.degrees(q_end[i]), 3) if i != 3 else round(q_end[3]*1000, 3) for i in range(5)])

t, q, qd, tau = simulate(q0, q_end, T, KP, KD, n=2000)

def disp(i, v):
    return v*1000 if i == 3 else np.degrees(v)

print('\n-- max |velocity| (deg/s or mm/s) --')
for i in range(5):
    print(i, round(np.max(np.abs([disp(i, v) for v in qd[:, i]])), 4))

print('\n-- max |torque| (N.m or N) --')
for i in range(5):
    print(i, round(np.max(np.abs(tau[:, i])), 4))

print('\n-- gravity hold at q_end G(q_end) --')
print(np.round(gravload(q_end), 4))

print('\n-- final joint error --')
ef = q[-1] - q_end
print([round(disp(i, ef[i]), 5) for i in range(5)])

ee = np.array([fk_position(v) for v in q])
ee_des = fk_position(q_end)
cart = np.linalg.norm(ee - ee_des, axis=1)*1000
print('\npeak cartesian err mm:', round(cart.max(), 4))
print('final cartesian err mm:', round(cart[-1], 5))
for tol in (1.0, 0.1):
    idx = np.argmax(cart < tol) if np.any(cart < tol) else None
    print(f'time to < {tol} mm:', round(t[idx], 3) if idx else 'never')

# manipulability range
w = [np.sqrt(max(np.linalg.det(robot.jacob0(qk)[:3] @ robot.jacob0(qk)[:3].T), 0)) for qk in q]
print('\nmanipulability min/max:', round(min(w), 4), round(max(w), 4))

# acceleration/jerk peaks from quintic reference
coeffs = quintic_coeffs(q0, q_end, T)
ts = np.linspace(0, T, 2000)
acc = np.array([quintic_eval(coeffs, tt)[2] for tt in ts])
vel = np.array([quintic_eval(coeffs, tt)[1] for tt in ts])
print('\n-- reference max |vel| / |acc| (deg or mm) --')
for i in range(5):
    print(i, 'vel', round(np.max(np.abs([disp(i, v) for v in vel[:, i]])), 4),
          'acc', round(np.max(np.abs([disp(i, a) for a in acc[:, i]])), 4))
