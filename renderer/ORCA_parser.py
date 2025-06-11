#!/usr/bin/env python3
import sys
from renderer import Renderer
import pygame

def parse_input(renderer):
    while True:
        line = input()  # o sys.stdin.readline()
        if line.startswith("Step"):
            renderer.update_with_step(line)
        else:
            renderer.configure(line)

def process_input_config(renderer):
    agents_default_param = {}
    error_lines = []
    agents_init_pos = []
    step = -1
    map = []
    state = "map"

    line = sys.stdin.readline().strip()
    valor = line.split(":", 1)[1].strip()
    height = int(valor)
    # renderer.start()

    for i, line in enumerate(sys.stdin):
        line = line.strip()
        if state == "map":
            row = line.split()
            map.append(row)
            if i >= height - 1:
                state = "param"
                continue
            # if i < height:
            #     row = line.split()
            #     map.append(row)
            # else:
            #     state = "param"
            #     continue

        if state == "param":
            if i < height + 7:
                clave, valor = line.split(":", 1)
                agents_default_param[clave.strip()] = valor.strip()
            else:
                state = "error"

        if state == "error":
            if line.startswith("NumAgents"):
                clave, valor = line.split(":", 1)
                agents_default_param[clave.strip()] = int(valor.strip())
                state = "agent"
                continue 
            error_lines.append(line)

        elif state == "agent":
            if line.startswith("Step"):
                state = "step"
                step += 1
                break
            id, start_xr, start_yr, goal_xr, goal_yr = line.split()
            agent = {
                "id": int(id),
                "start": (float(start_xr), float(start_yr)),
                "goal": (float(goal_xr), float(goal_yr))
            }
            agents_init_pos.append(agent)
    
    title = f"Simulator ORCA - {agents_default_param['NumAgents']} agents"
    renderer.set_title(title)
    renderer.sightRadius = float(agents_default_param.get('sightRadius', -1))
    renderer.radius = float(agents_default_param.get('radius', -1))
    # renderer.set_agent_default_param(agents_default_param)
    renderer.set_agents_init_pos(agents_init_pos)
    renderer.set_map(map)


if __name__ == "__main__":

    #TO DO: GET WORLD NAME
    pygame.init()
    renderer = Renderer(800, 600, cell_size=50)
    # renderer.set_title("Simulator ORCA")
    # renderer.start()
    process_input_config(renderer)
    
    renderer.start()

    for item, value in renderer.get_agents_default_param().items():
        print(f"{item} {value}")

    for agent in renderer.get_agents_init_pos():
        print(f"Agent {agent['id']} \t {agent['start'][0]} {agent['start'][1]} \t {agent['goal'][0]} {agent['goal'][1]}")

    # while True:
    #     for i in range(agents_default_param['NumAgents']):
    #         line = sys.stdin.readline().strip()
            
    #         posX, posY = line.split(" ")
    #         print(f"Agent {i} \t id: {agents_init_pos[i]['id']} \t {posX} {posY}")
    #     line = sys.stdin.readline().strip()
    #     if not line:
    #         break

    while True:
        for agent in renderer.get_agents_init_pos():
            line = sys.stdin.readline().strip()
            posX, posY = line.split(" ")
            print(f"Agent {agent['id']} \t {posX} {posY}")

        line = sys.stdin.readline().strip()
        if not line:
            break

    pygame.quit()
    sys.exit(0)
        
        




