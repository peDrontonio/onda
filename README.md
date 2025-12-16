# Manipulador Robótico com Controle Cinemático e Grasping

Projeto Onda da disciplina de Dinâmica de Sistemas Robóticos (SEM0590)

## Descrição

Este projeto implementa manipulador robótico focado em tarefas de solda. O objetivo é demonstrar conceitos avançados de cinemática direta e inversa, dinâmica de sistemas robóticos e controle de movimento em ambientes simulados e reais. O sistema resolve problemas de planejamento de trajetória e aplicações industriais e de pesquisa.

A cinemática é tratada utilizando a Robotics Toolbox de Peter Corke, implementada em Python, permitindo cálculos precisos de posições e orientações das juntas. A dinâmica é considerada para simulações realistas, incluindo efeitos de gravidade e inércia. O controle é baseado em algoritmos de controle direto, integrando feedback de sensores para ajustes em tempo real.

## Funcionalidades

- **Controle de Movimento**: Implementação de cinemática direta e inversa para posicionamento preciso das juntas do manipulador.
- **Simulação e Visualização**: Integração com simuladores como Gazebo e RViz para testes virtuais e visualização 3D do movimento.
- **Interface com Hardware**: Comunicação via ROS (Robot Operating System) e MoveIt para controle de robôs reais.
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

1. Para usar Docker (recomendado):
   - Construa a imagem: `./scripts/build.sh`
   - Execute o container: `./scripts/run.sh`

### Configuração com Docker e Execução

Para um ambiente isolado e fácil de configurar, use Docker:

1. **Construir a Imagem Docker**:
   ```bash
   ./scripts/build.sh
   ```
   Este comando constrói uma imagem baseada no ROS Humble com todas as dependências necessárias.

2. **Executar o Container**:
   ```bash
   ./scripts/run.sh
   ```
   Isso inicia um container interativo onde você pode executar comandos ROS e scripts Python.

3. **Executar Códigos de Exemplo**:
   Dentro do container ou no ambiente local, execute demonstrações:
   - Demo básica: `python peter_corke/scripts/demo.py`
   - Cinemática direta: `python peter_corke/scripts/cinematica_direta.py`
   - Cinemática inversa: `python peter_corke/scripts/cinematica_inversa.py`
   - Visualização: `python peter_corke/scripts/braco_swift_visualization.py`

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
   ros2 launch braco_description display.launch.py
   ```
2. Lance o planejador de trajetória.
   ```bash
   ros2 run braco_desciption trajectory.launch.py
   ```

### Cĺculo e implementação das cinemáticas

- Use scripts Python para definir posições das juntas:
  ```python
  from roboticstoolbox import DHRobot, RevoluteDH
  # Defina o robô e calcule cinemática
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


## Referências e Links Úteis

- [ROS Documentation](https://docs.ros.org/en/humble/)
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
- [MoveIt Tutorials](https://moveit.picknik.ai/humble/)
- [Gazebo Simulation](https://gazebosim.org/)
- [Peter Corke Robotics Book](https://petercorke.com/robotics/)

## Imagens e Vídeos

![Simulação](https://drive.google.com/file/d/1Bioajuh18dL6YXh0Bl7cxDEz-S56hfDP/view?usp=sharing)
*Simulação do manipulador executando uma trajetória*

[Vídeo Demonstrativo](https://drive.google.com/file/d/1YR7aKbvposASezvE1vFXXejYeSh8mhne/view?usp=sharing) 
