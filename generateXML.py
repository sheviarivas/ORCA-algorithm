import random
import xml.etree.ElementTree as ET
from xml.dom import minidom

def prettify(elem):
    """Devuelve un XML con formato bonito y legible"""
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def generar_xml(nombre_archivo):
    # Crear el elemento raíz
    root = ET.Element("root")
    numAgents = 4
    generate_agents(root, numAgents=numAgents)

    grid = []
    width, height = 16, 16
    generate_grid(root, width, height, grid)
    register_map(root, width, height, grid)

    genetare_algorithm(root)

    # Convertir a XML con formato legible
    xml_string = prettify(root)

    # Guardar el archivo
    with open(nombre_archivo, "w", encoding="utf-8") as f:
        f.write(xml_string)
    
def register_map(root, width, height, grid):
    tag_map = ET.SubElement(root, "map")
    tag_width = ET.SubElement(tag_map, "width").text = str(width)
    tag_height = ET.SubElement(tag_map, "height").text = str(height)
    tag_cellsize = ET.SubElement(tag_map, "cellsize").text = "1"

    tag_grid = ET.SubElement(tag_map, "grid")
    for row in grid:
        tag_row = ET.SubElement(tag_grid, "row").text = ' '.join(map(str, row))


def register_obstacles(root, numObstacles):
    agents = ET.SubElement(root, "agents", number=str(numObstacles))

    pass

def generate_agents(root, numAgents):
    agents = ET.SubElement(root, "agents", number=str(numAgents), type="orca-par")
    default_parameters = ET.SubElement(agents, "default_parameters", size="0.3", movespeed="2.1", 
                                       agentsmaxnum="10", timeboundary="5.4", sightradius="3.0", timeboundaryobst="33")
    
    # Register agents
    agent_dict = {}

    for i in range(numAgents):
        agent_dict["id"] = str(i)
        agent_dict["start.xr"] = str(i*2)
        agent_dict["start.yr"] = str(i*2)
        agent_dict["goal.xr"] = str(i*2)
        agent_dict["goal.yr"] = str(i*2)
        agent = ET.SubElement(agents, "agent", id=str(i), attrib= agent_dict)

def generate_grid(root, width, height, grid: list):
    for y in range(height):
        row = [0]*width
        grid.append(row)

    num_obst = 3
    # num_obst = random.randint(0, width*height)
    tag_obstacles = ET.SubElement(root, "obstacles", number=str(num_obst))

    while num_obst:
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        if not grid[y][x]:
            grid[y][x] = 1
            num_obst -= 1
            generate_square_obstacle(tag_obstacles, (x, height - 1 - y - 1))
    
    pass

def generate_square_obstacle(tag_obstacles, bottom_left: tuple):
    square_points = [bottom_left, (bottom_left[0] + 1, bottom_left[1]), (bottom_left[0] + 1, bottom_left[1] + 1), (bottom_left[0], bottom_left[1] + 1)]
    tag_obstacle = ET.SubElement(tag_obstacles, "obstacle")
    for point in square_points:
        tag_vertex = ET.SubElement(tag_obstacle, "vertex", xr=str(point[0]), yr=str(point[1]))
    
    pass

def generate_map():
    pass

def genetare_algorithm(root):
    tag_algorithm = ET.SubElement(root, "algorithm")

    tag_searchtype = ET.SubElement(tag_algorithm, "searchtype").text = "thetastar"
    tag_breakingties = ET.SubElement(tag_algorithm, "breakingties").text = "0"
    tag_allowsqueeze = ET.SubElement(tag_algorithm, "allowsqueeze").text = "false"
    tag_cutcorners = ET.SubElement(tag_algorithm, "cutcorners").text = "false"
    tag_hweight = ET.SubElement(tag_algorithm, "hweight").text = "1"
    tag_timestep = ET.SubElement(tag_algorithm, "timestep").text = "0.1"
    tag_delta = ET.SubElement(tag_algorithm, "delta").text = "0.1"
    tag_trigger = ET.SubElement(tag_algorithm, "trigger").text = "common-point"
    tag_mapfnum = ET.SubElement(tag_algorithm, "mapfnum").text = "3"

def register_agent():
    pass

def register_obstacle():
    pass


if __name__ == "__main__":
    generar_xml("salida.xml")
    print("Archivo XML generado exitosamente.")
