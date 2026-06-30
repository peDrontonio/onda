# Braço Robótico RRRPR — Controle por Torques

Projeto da disciplina **SEM0590 — Dinâmica de Sistemas Robóticos**  
Universidade de São Paulo

---

## Visão Geral

Implementação completa de um manipulador robótico de 5 graus de liberdade (cadeia RRRPR) com:

- Cinemática direta e inversa derivadas diretamente do URDF
- Planejamento de trajetória por **polinômio quíntico** (C² contínuo)
- **Controlador PD com compensação de gravidade** — o controlador recebe uma posição desejada e computa os torques necessários: `τ = Kp·e + Kd·ė + G(q)`
- Simulação em malha fechada da dinâmica completa (`M·q̈ + C·q̇ + G = τ`)
- Integração com **Gazebo Harmonic** via `gz_ros2_control` e interface de esforço

---

## Estrutura do Projeto

```
onda/
├── peter_corke/scripts/
│   ├── cinematica_inversa.py   — FK/IK (modelo de referência)
│   ├── closed_loop_sim.py      — simulação dinâmica em malha fechada (standalone)
│   └── position_ui.py          — visualizador 3D interativo
│
└── ros_ws/src/
    ├── braco_description/
    │   ├── urdf/Braço.xacro           — modelo URDF com interface de esforço
    │   ├── config/controllers.yaml    — effort_controller (JointGroupEffortController)
    │   ├── launch/gazebo.launch.py    — Gazebo Harmonic com controlador PD+G
    │   ├── scripts/
    │   │   ├── braco_controller.py    — nó ROS2: PD + compensação de gravidade
    │   │   ├── trajectory_planner.py  — IK + polinômio quíntico + gráficos
    │   │   └── send_cartesian.py      — envia posição cartesiana via IK
    │   └── braco_description/
    │       └── cinematica_inversa.py  — módulo Python (importável pelos nós ROS)
    └── manipulator_gazebo/
        └── launch/spawn_braco.launch.py
```

---

## Cadeia Cinemática

O manipulador possui a cadeia **RRRPR** (base → efetuador):

| Junta | Tipo | Eixo | Origem (rel. pai) | Limites |
|-------|------|------|-------------------|---------|
| `base_rot1`   | Revoluta  | Z  | (0, 0, 0.035) m     | [0, 2π] |
| `rot1_rot2`   | Revoluta  | X  | (0.1, 0, 0.065) m   | [0, 2π] |
| `rot2_rot3`   | Revoluta  | Y  | (0.08, 0.1, 0) m    | [0, 2π] |
| `rot3_prism1` | Prismática | Y  | (-0.18, 0.1, 0) m   | [0, 0.1 m] |
| `prism1_rot4` | Revoluta  | -X | (0, 0.165, 0) m     | [±60°] |

**Posição de repouso** (q = 0): efetuador em (0, 0.365, 0.1) m no frame mundo.

### Cinemática Direta

FK implementada como produto de transformadas homogêneas derivadas diretamente do URDF
(sem aproximação por parâmetros DH):

```
T(q) = Tz(q1) · Tx_y(0.1,0,0.065) · Rx(q2) · ...
```

### Cinemática Inversa

IK numérica com otimizador SLSQP (scipy), múltiplos pontos de partida, tolerância < 1 mm.

---

## Lei de Controle

O controlador implementa **PD com compensação de gravidade**:

```
τ = Kp·(q_des − q) + Kd·(q̇_des − q̇) + G(q)
```

| Ganho | Juntas [θ₁, θ₂, θ₃] | Prismatic d₄ | Junta θ₅ |
|-------|---------------------|--------------|----------|
| **Kp** | 120, 100, 80 N·m/rad | 500 N/m | 30 N·m/rad |
| **Kd** | 10, 8, 6 N·m·s/rad  | 30 N·s/m | 2 N·m·s/rad |

**G(q)** usa as massas reais do URDF:

```
G[1] = (m₂+m₃+m₄)·g·0.1·cos(q₂)
G[2] = (m₃+m₄)·g·0.08·cos(q₂+q₃)
G[3] = m₄·g·sin(q₂+q₃)
G[4] = 0.01·g·cos(q₅)
```

massas [m₁…m₆] = [1.91, 6.85, 7.27, 14.51, 4.19, 0.65] kg (do URDF).

---

## Simulação em Malha Fechada (standalone)

Integra a dinâmica completa sem Gazebo — roda em qualquer máquina com Python 3:

```bash
cd peter_corke/scripts
python3 closed_loop_sim.py -0.155 0.290 0.361 --duration 4.0 --output output/
```

Saída: `output/tracking.png`, `output/tracking_error.png`, `output/torques.png`, `output/summary.png`

A simulação integra `q̈ = M(q)⁻¹·[τ − C(q,q̇) − G(q)]` com o controlador PD+G acima.
Convergência verificada: erro cartesiano final < 0.001 mm.

---

## Visualizador Interativo

```bash
cd peter_corke/scripts
python3 position_ui.py
```

Interface matplotlib com caixa de texto para entrada de X, Y, Z e visualização 3D animada do braço.

---

## Simulação no Gazebo

### Dependências (instalar uma vez)

```bash
sudo apt install \
    ros-humble-gz-ros2-control \
    ros-humble-effort-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-xacro
```

### Build

```bash
cd ros_ws
colcon build
source install/setup.bash
```

### Executar

```bash
# Janela 1 — Gazebo + controlador PD+G
ros2 launch braco_description gazebo.launch.py

# Janela 2 — planejar e enviar trajetória
ros2 run braco_description trajectory_planner.py -0.155 0.290 0.361 --duration 4.0

# Alternativa: enviar posição cartesiana direta
ros2 run braco_description send_cartesian.py -0.155 0.290 0.361
```

### Arquitetura dos nós

```
trajectory_planner.py ──┐
                         ├─► /braco/trajectory_command (JointTrajectory)
send_cartesian.py ───────┘
                                    ▼
                         braco_controller.py   ← /joint_states
                                    │
                                    ▼
                         /effort_controller/commands (Float64MultiArray)
                                    ▼
                    effort_controllers/JointGroupEffortController
                                    ▼
                              gz_ros2_control
                                    ▼
                              Gazebo Harmonic
```

O `trajectory_planner.py` também gera gráficos de posição, velocidade, aceleração e torques de dinâmica inversa na pasta `output/`.

---

## Referências

- Craig, J. J. — *Introduction to Robotics: Mechanics and Control*
- ROS 2 Humble Docs — https://docs.ros.org/en/humble/
- Gazebo Harmonic — https://gazebosim.org/
- gz_ros2_control — https://github.com/ros-controls/gz_ros2_control
