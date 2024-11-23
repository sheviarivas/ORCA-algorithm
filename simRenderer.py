#!/usr/bin/python3

import subprocess
import yaml
import os
import sys

# Llamar al ejecutable a.out
def run_simulation(program, file, nAgents):
    print("Running simulation...")

    result = subprocess.run([program, 
                             file, 
                             nAgents
                             ], capture_output=True, text=True)

    output_file = "steps.yaml"
    with open(output_file, 'w') as f:
        f.write(result.stdout)
    
    # Imprimir la salida del programa C++
    # print("Salida del programa C++:")
    # print(result.stdout)
    
    # Si hubo errores
    if result.stderr:
        print("Error en el programa C++:")
        print(result.stderr)
        return -1
    
    
def run_renderer(XML_file):
    # llamar a YAML4renderer para actualizar combined.yaml quien será llamado por el renderer
    print("steps done!")
    print("creating YAML...")


    print("Running renderer...")

    env = os.environ.copy()
    env["PYTHONPATH"] = "/home/neopren/repos/Python-RVO2-SimRender:" + env.get("PYTHONPATH", "")

    workspace_renderer = "/home/neopren/repos/Python-RVO2-SimRender"
    renderer_program = "simulator/engines/RVO2SimulatorWrapper_heviav02.py"

    command = [ "python", 
                "YAML4renderer.py",
                XML_file
    ]

    result = subprocess.run(command, capture_output=True, text=True, cwd=workspace_renderer)
    print(result.stdout)

    # if not YAML from XML created:
        # create
        # join steps from simulator to YAML
    YAML_file = "combined.yaml"
    
    result = subprocess.run(["python",
                             renderer_program,
                             YAML_file
                            ], env=env, cwd=workspace_renderer, capture_output=True, text=True)

    # Imprimir la salida del programa C++
    # print("Salida del programa C++:")
    # print(result.stdout)
    
    # Si hubo errores
    if result.stderr:
        print("Error en el programa python:")
        print(result.stderr)


if __name__ == "__main__":


    # Crear programa para generar el xml | Leer xml existente
    # Extraer info necesaria del mapa obtenido para el renderer
    # Correr simulación y obtener steps y submapas
    # Correr el renderer
    
    workspace_simulation = "/home/neopren/repos/ORCA-algorithm"
    program =   workspace_simulation + "/vanillaBuild/single_test"
    XML_file =      workspace_simulation + "/paper_tasks/interesting/" + sys.argv[1]
    nAgents =   sys.argv[2]

    run_simulation(program, XML_file, nAgents)
    run_renderer(XML_file)

