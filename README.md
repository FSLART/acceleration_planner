# acceleration

Planner ROS 2 para a prova de *acceleration* (reta de 75 m).

Nó: `acceleration_node` (executável `acceleration_exec`).

## Comportamento

1. No arranque, carrega `share/acceleration/data/acceleration_path.csv` (reta de 75 m, ponto a
   cada 0.5 m) para a memória.
2. Enquanto o mapa não está localizado, subscreve `/mapping/cones` e procura o cone azul e o
   cone amarelo mais próximos do carro. A cada deteção válida (largura de pista plausível),
   acumula a posição média entre os dois cones e o heading perpendicular ao par. Depois de
   `LOCALIZATION_SAMPLES` deteções, calcula a média e roda/translada o mapa carregado do CSV
   para a linha de partida real — o mapa fica então "trancado".
3. Depois de localizado, a cada pose recebida em `/slam/pose`, publica o troço do caminho a
   partir do ponto mais próximo do carro até ao fim da reta (sem *wraparound*, já que a prova
   não é um circuito fechado) em `/path` (`lart_msgs/msg/PathSpline`) e `/path/markers`
   (`nav_msgs/msg/Path`, para visualização).

## Tópicos

- Subscreve: `/mapping/cones` (`lart_msgs/msg/ConeArray`), `/slam/pose` (`geometry_msgs/msg/PoseStamped`)
- Publica: `/path` (`lart_msgs/msg/PathSpline`), `/path/markers` (`nav_msgs/msg/Path`)
