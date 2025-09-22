# Manipulador Robótico com Controle Cinemático e Grasping

Projeto Onda da disciplina de Dinâmica de Sistemas Robóticos (SEM0590)

## Descrição

Este projeto implementa um sistema de controle para um manipulador robótico focado em tarefas de grasping (pegada) e manipulação de objetos. O objetivo é demonstrar conceitos avançados de cinemática direta e inversa, dinâmica de sistemas robóticos e controle de movimento em ambientes simulados e reais. O sistema resolve problemas de planejamento de trajetória, detecção de colisões e otimização de pegada para aplicações industriais e de pesquisa.

A cinemática é tratada utilizando a Robotics Toolbox de Peter Corke, implementada em Python, permitindo cálculos precisos de posições e orientações das juntas. A dinâmica é considerada para simulações realistas, incluindo efeitos de gravidade e inércia. O controle é baseado em algoritmos de controle direto, integrando feedback de sensores para ajustes em tempo real.

## Funcionalidades

- **Controle de Movimento**: Implementação de cinemática direta e inversa para posicionamento preciso das juntas do manipulador.
- **Detecção e Desvio de Obstáculos**: Algoritmos para planejamento de trajetória que evitam colisões com objetos no ambiente.
- **Simulação e Visualização**: Integração com simuladores como Gazebo e RViz para testes virtuais e visualização 3D do movimento.
- **Interface com Hardware**: Comunicação via ROS (Robot Operating System) e MoveIt para controle de robôs reais.
- **Grasping Avançado**: Classes especializadas para cálculo de pegadas ótimas, considerando geometria de objetos e força aplicada.
- **Demonstrações Interativas**: Scripts Python para testes rápidos e validação de algoritmos.

## Tecnologias Usadas

- **ROS Humble**: Framework principal para robótica, utilizado para comunicação entre nós, controle de hardware e simulação.
- **Python**: Linguagem principal para scripts de controle, simulação e análise.
- **Robotics Toolbox (Peter Corke)**: Biblioteca para cálculos de cinemática, dinâmica e controle de robôs.
- **Gazebo**: Simulador físico para testes de movimento e interação com o ambiente.
- **RViz**: Ferramenta de visualização para monitoramento do estado do robô em tempo real.
- **Docker**: Containerização para facilitar a instalação e execução em diferentes sistemas.
- **MoveIt**: Framework para planejamento de movimento e manipulação em ROS.

## Instalação

### Requisitos de Sistema

- Ubuntu 20.04 ou superior
- ROS Humble instalado
- Python 3.8+
- Docker (opcional, mas recomendado para isolamento)

### Dependências

1. Instale ROS Humble e pacotes essenciais:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop ros-humble-moveit ros-humble-gazebo-ros-pkgs
   ```

2. Instale dependências Python:
   ```bash
   pip install roboticstoolbox-python spatialmath-python numpy matplotlib
   ```

3. Para usar Docker (recomendado):
   - Construa a imagem: `./scripts/build.sh`
   - Execute o container: `./scripts/run.sh`

### Compilação e Execução

1. Clone o repositório e navegue para o diretório:
   ```bash
   git clone <url-do-repositorio>
   cd onda
   ```

2. Configure o workspace ROS:
   ```bash
   cd ros_ws
   colcon build
   source install/setup.bash
   ```

3. Execute uma demonstração básica:
   ```bash
   python peter_corke/scripts/demo.py
   ```

## Como Usar

### Iniciando a Simulação

1. Lance o Gazebo com o modelo do robô:
   ```bash
   ros2 launch <pacote> gazebo.launch.py
   ```

2. Abra o RViz para visualização:
   ```bash
   ros2 run rviz2 rviz2
   ```

### Controlando o Manipulador

- Use scripts Python para definir posições das juntas:
  ```python
  from roboticstoolbox import DHRobot, RevoluteDH
  # Defina o robô e calcule cinemática
  ```

- Para grasping, utilize a classe GraspeClass:
  ```python
  from graspeClass import GraspeClass
  gripper = GraspeClass()
  gripper.grasp_object(position, orientation)
  ```

### Visualização

No RViz, adicione displays para o modelo do robô, trajetórias e sensores. Use tópicos ROS para publicar comandos de movimento.

## Exemplo de Simulação/Execução

Aqui está um exemplo simples de execução de uma trajetória básica:

```python
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH

# Defina o robô (exemplo com 3 juntas)
robot = DHRobot([
    RevoluteDH(d=0.1, a=0.5),
    RevoluteDH(alpha=np.pi/2, a=0.4),
    RevoluteDH(d=0.2)
], name="MyRobot")

# Posição desejada
T_desired = robot.fkine([0.5, 0.3, 0.1])  # Cinemática direta

# Calcule juntas para alcançar a posição (cinemática inversa)
q = robot.ikine_LM(T_desired)

print("Juntas calculadas:", q.q)
```

**Entrada Esperada**: Posição e orientação desejada do efetuador.
**Saída**: Valores das juntas do robô para alcançar a pose.

Para executar: `python exemplo.py`

## Arquitetura do Sistema

O sistema é dividido em módulos ROS:

- **Sensores**: Nós para leitura de dados de câmeras, força e posição.
- **Controlador**: Algoritmos de cinemática e dinâmica para planejamento de movimento.
- **Atuadores**: Interface com motores e gripper para execução física.

Fluxo de Dados:
1. Sensores → Processamento (cinemática inversa) → Planejamento de Trajetória → Controle de Atuadores.

Integração: ROS actua como middleware, conectando simulação (Gazebo) com hardware real via MoveIt.

## Testes

### Testes Unitários

Execute testes para funções de cinemática:
```bash
python -m pytest tests/test_kinematics.py
```

### Testes de Integração

Lance simulação completa no Gazebo e verifique trajetórias:
```bash
ros2 launch <pacote> test_integration.launch.py
```

### Simulações

Para testar grasping: `python peter_corke/scripts/demo_first_try.py`

## Contribuição

Contribuições são bem-vindas! Para contribuir:

1. Fork o repositório.
2. Crie uma branch para sua feature: `git checkout -b feature/nova-funcionalidade`
3. Commit suas mudanças: `git commit -m 'Adiciona nova funcionalidade'`
4. Push para a branch: `git push origin feature/nova-funcionalidade`
5. Abra um Pull Request.

Siga as convenções de código e adicione testes para novas funcionalidades.

## Licença

Este projeto está licenciado sob a MIT License. Veja o arquivo LICENSE para detalhes.

## Referências e Links Úteis

- [ROS Documentation](https://docs.ros.org/en/humble/)
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
- [MoveIt Tutorials](https://moveit.picknik.ai/humble/)
- [Gazebo Simulation](https://gazebosim.org/)
- [Peter Corke Robotics Book](https://petercorke.com/robotics/)

## Imagens e Vídeos

![Simulação no Gazebo](docs/images/gazebo_simulation.png)
*Simulação do manipulador executando uma tarefa de grasping.*

[Vídeo Demonstrativo](https://example.com/video) (placeholder - adicione link real)

## Problemas Conhecidos

- **Limitação na Cinêmica Inversa**: Algumas poses podem não ter solução única; use métodos numéricos como LM para aproximar.
- **Compatibilidade com Hardware**: Testado principalmente em simulação; ajustes necessários para robôs reais devido a folgas mecânicas.
- **Performance**: Simulações pesadas podem exigir hardware potente; considere otimizar loops de controle.

Para soluções alternativas, consulte a documentação do MoveIt para planejamento avançado.
