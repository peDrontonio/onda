# Braço Robótico RRRPR — Controle por Torques

Projeto da disciplina **SEM0590 — Dinâmica de Sistemas Robóticos**  
Universidade de São Paulo

---

## Visão Geral

Implementação completa de um manipulador robótico de 5 graus de liberdade (cadeia RRRPR) com:

- Modelo cinemático **e dinâmico** carregado no **Robotics Toolbox (Peter Corke)** diretamente do URDF do robô (massas, centros de massa e tensores de inércia reais — sem aproximação por parâmetros DH nem inércias ajustadas à mão)
- Cinemática inversa por **Levenberg–Marquardt** (`ikine_LM`, posição) e direta por `fkine`
- Planejamento de trajetória por **polinômio quíntico** (C² contínuo)
- **Controlador PD com compensação de gravidade** — o controlador recebe uma posição desejada e computa os torques necessários: `τ = Kp·e + Kd·ė + G(q)`, com `G(q)` obtido do Robotics Toolbox
- Simulação em malha fechada da dinâmica completa (`M·q̈ + C·q̇ + G = τ`) integrada com solver rígido (SciPy `LSODA`)
- Integração com **Gazebo Harmonic** via `gz_ros2_control` e interface de esforço

---

## Estrutura do Projeto

```
onda/
├── peter_corke/
│   ├── requirements.txt        — dependências Python (Robotics Toolbox etc.)
│   └── scripts/
│       ├── braco_rtb.py            — modelo Robotics Toolbox (carrega o URDF: cinemática + dinâmica)
│       ├── cinematica_inversa.py   — FK/IK de referência (implementação própria, sem RTB)
│       ├── closed_loop_sim.py      — simulação dinâmica em malha fechada (RTB, standalone)
│       └── position_ui.py          — visualizador 3D interativo (RTB)
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

O modelo é carregado no **Robotics Toolbox** a partir do URDF (`braco_rtb.py`),
e a FK usa `robot.fkine(q)`. Em `q = 0` o efetuador fica em `(0, 0.365, 0.1)`,
conforme o URDF. Uma implementação própria equivalente (produto de transformadas
homogêneas, sem DH) está em `cinematica_inversa.py` como referência.

### Cinemática Inversa

IK por Levenberg–Marquardt do Robotics Toolbox (`robot.ikine_LM`, apenas
posição — máscara `[1,1,1,0,0,0]`, pois o braço tem 5 DOF para uma tarefa de
posição 3D), tolerância < 1 mm. A referência `cinematica_inversa.py` usa SLSQP
(scipy) com múltiplos pontos de partida.

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

## Ambiente Python (Robotics Toolbox)

Os scripts em `peter_corke/scripts/` usam o **Robotics Toolbox**. Crie um
ambiente virtual isolado (evita conflitos com pacotes do sistema/ROS) e instale
as dependências:

```bash
cd peter_corke
python3 -m venv .venv          # se faltar: sudo apt install python3.10-venv
                               # alternativa sem apt: python3 -m virtualenv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

O modelo é carregado do URDF (`ros_ws/src/braco_description/urdf/braco.urdf`) em
`braco_rtb.py`. Verifique o modelo com:

```bash
python scripts/braco_rtb.py     # imprime a cadeia RRRPR, massas e um teste de IK
```

## Simulação em Malha Fechada (standalone)

Integra a dinâmica completa (obtida do Robotics Toolbox) sem Gazebo:

```bash
cd peter_corke/scripts
python closed_loop_sim.py -0.155 0.290 0.361 --duration 4.0 --output output/
```

Saída (os quatro gráficos exigidos + resumo):
`output/tracking.png`, `output/velocity.png`, `output/tracking_error.png`,
`output/torques.png`, `output/summary.png`

A simulação integra `q̈ = M(q)⁻¹·[τ − C(q,q̇) − G(q)]` com o controlador PD+G
acima, usando `M`, `C·q̇+G` e `G` calculados pelo Robotics Toolbox (Newton–Euler
recursivo) e o solver rígido `LSODA` (o laço de alto ganho é rígido).
Convergência verificada: erro cartesiano final ≈ 0.002 mm.

---

## Visualizador Interativo

```bash
cd peter_corke/scripts
python position_ui.py      # com o venv ativado
```

Interface matplotlib com caixa de texto para entrada de X, Y, Z e visualização 3D animada do braço (cinemática do Robotics Toolbox).

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
