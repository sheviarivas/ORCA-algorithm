import sys
import subprocess
import os
import argparse

# calls renderer.py

# python wrapper.py ORCA|RVO2 world nAgents -log OFF|ON -renderer OFF|ON 


def str2bool(value):
    if value.lower() in ('true', '1', 't', 'y', 'yes'):
        return True
    elif value.lower() in ('false', '0', 'f', 'n', 'no'):
        return False
    raise ValueError('Debe ser un valor booleano: true/false.')

if __name__ == "__main__":

    engine_default = "ORCA"

    parser = argparse.ArgumentParser()
    parser.add_argument('-w', '--world',   required=True, type=str)
    parser.add_argument('-n', '--nAgents', required=True, type=str)
    parser.add_argument('-r', '--render',  type= str2bool, default=True)
    parser.add_argument('-l', '--log',     type= str2bool, default=True)
    parser.add_argument('-e', '--engine',  type=str, default=engine_default)

    args = parser.parse_args()

    engine_options = ['ORCA', 'RVO2']

    if args.engine not in engine_options:
        print(f"⚠️ Valor inválido: {args.engine}. Usando valor por defecto: {engine_default}")
        args.engine = engine_default

    workspace = "/home/neopren/repos/ORCA-algorithm"
    world = workspace + "/paper_tasks/interesting/" + args.world + ".xml"

    # Inicia programa1 y captura su stdout
    simulation_path = workspace + "/build_renderer/single_test"
    p1 = subprocess.Popen([simulation_path, world, args.nAgents], stdout=subprocess.PIPE)

    # Inicia renderer.py, conectando su stdin al stdout de programa1
    p2 = subprocess.Popen(["python3", "ORCA_parser.py"], stdin=p1.stdout)

    # Cierra la salida de programa1 en el contexto del padre
    p1.stdout.close()

    # Espera a que programa2 termine
    p2.wait()


    

