#include "thetastar.h"

ThetaStar::ThetaStar(const Map &map, const environment_options &options, const Point &start, const Point &goal,
					 const float &radius)
		: PathPlanner(map, options, start, goal, radius) {
	currPath = std::list<Point>();
	close = std::unordered_map<int, Node>();
	open = std::vector<std::list<Node>>();
	openSize = 0;
	glPathCreated = false;
	visChecker = LineOfSight(this->radius / map.GetCellSize());
	past = glStart;
}


ThetaStar::ThetaStar(const ThetaStar &obj) : PathPlanner(obj) {
	currPath = obj.currPath;
	close = obj.close;
	open = obj.open;
	openSize = obj.openSize;
	glPathCreated = obj.glPathCreated;
	visChecker = obj.visChecker;
	past = obj.past;

}


ThetaStar::~ThetaStar() {}


bool ThetaStar::GetNext(const Point &curr, Point &next) {
	if (glPathCreated) {
		if (currPath.size() == 0) {	// quizá el caso en que inicialicé el ag en su propia meta, debo probarlo... // o quizá cuando ya llega a la meta, pero para que se quede ahí
			currPath.push_back(glGoal);
		}

		if (currPath.size() > 1) {	// revisar!!	// currPath posee las celdas a las que el ag debe ir, sin considerar su pos actual
			float sqDistToCurr = (currPath.front() - curr).SquaredEuclideanNorm();	// xa*xb + ya*yb
			float sqDelta = options->delta * options->delta; // TODO Separate option
			if (sqDistToCurr < sqDelta) {	// if dentro de una meta local

				if (!(currPath.front() == this->glGoal)) {
					past = currPath.front();	// quizá past es una variable para dsp?? // punto previo
				}
				currPath.pop_front();
				next = currPath.front();
				return true;
			}
		}
		// fuera de una meta local (no la alcanzo por distancia delta establecida)
		Node currNode = map->GetClosestNode(curr);	// obtener centro del nodo actual

		Node nextNode = map->GetClosestNode(currPath.front());	// siguiente nodo en el path
		if (std::next(currPath.begin()) != currPath.end()) {	// verdadero si size!=1	// But why though? // Cuando size == 1 quiere decir que solo le queda llegar a la meta final
			Node nextNextNode = map->GetClosestNode(*std::next(currPath.begin()));
			if (currNode == nextNode || visChecker.checkLine(currNode.i, currNode.j, nextNode.i, nextNode.j, *map)) {	// 1.por si no tengo que llegar a mi meta actual, llegué a la sig. a esa 2. similar a 1. puedo llegar a la sig. meta desde dd toi
				next = currPath.front();
				if (currNode == nextNextNode ||
					visChecker.checkLine(currNode.i, currNode.j, nextNextNode.i, nextNextNode.j, *map)) {	// 1. igual que el anterior. 2. compara con la versión node de next
					currPath.pop_front();	// cambia el next por el siguiente. Quiere decir que alcancé la meta actual? a pesar de no cumplir con la distancia anterior?
					next = currPath.front();
				}
				return true;
			}
		}
		else {	// verdadero con 0 o 1 elementos (no muy seguro del 0), aunque se hacen cargo del caso size == 0 antes, más arriba, so idk. // Este es el caso cuando solo queda la meta final
			if (currNode == nextNode || visChecker.checkLine(currNode.i, currNode.j, nextNode.i, nextNode.j, *map)) {
				next = currPath.front();
				return true;
			}
		}


//        Node currNode = map->GetClosestNode(curr), nextNode = map->GetClosestNode(currPath.front());
//        if(currNode == nextNode || visChecker.checkLine(currNode.i, currNode.j, nextNode.i, nextNode.j, *map))
//        {
//            next = currPath.front();
//            return true;
//        }

		Point last = currPath.front();	// no se usa
		currPath.pop_front();			// no se usa

//        bool isLastAccessible = SearchPath(map->GetClosestNode(curr), map->GetClosestNode(last));
//        if(isLastAccessible)
//        {
//            next = currPath.front();
//            return true;
//        }
		currPath.clear();	// se recalcula todo otra vez?	quiero corroborarlo con un ejemplo
		bool isGoalAccessible = SearchPath(map->GetClosestNode(curr), map->GetClosestNode(glGoal));
		if (isGoalAccessible) {
			next = currPath.front();
			return true;
		}


	}
	return false;
}


bool ThetaStar::SearchPath(const Node &start, const Node &goal) {
	// si ya tiene, se borra todo lo auxiliar
	if (glPathCreated) {
		close.clear();
		for (auto &l: open) {
			l.clear();
		}
		openSize = 0;
	}
	else {
		// por qué height? a lo mjr recorre desde height filas, fila por fila. Asies
		open.resize(map->GetHeight());
	}

	Node curNode;
	curNode.i = start.i;	// valor height - y. Why? idk (quizá para el simulador de RVO2)
	curNode.j = start.j;	// valor x
	curNode.g = 0;

	// heuristic
	curNode.H = ComputeHFromCellToCell(curNode.i, curNode.j, goal.i, goal.j);

	// investigar options->hweight. Aparece mencionado en el repo <hweight>
	curNode.F = options->hweight * curNode.H;									
	curNode.parent = nullptr;
	AddOpen(curNode);
	int closeSize = 0;
	bool pathfound = false;
	while (!StopCriterion()) {	// mientras queden elementos en la openList...
		curNode = FindMin();	// obtiene el nodo con el F más pequeño de la lista abierta
		close.insert({curNode.i * map->GetWidth() + curNode.j, curNode});	// qué representará curNode.i * map->GetWidth() + curNode.j... será un ID para cada nodo??... Creo que yes
		closeSize++;
		open[curNode.i].pop_front();	// el que está en frente será el de menor F? Revisar AddOpen(). Toi casi seguro que es así
		openSize--;
		if (curNode == goal) {
			pathfound = true;
			break;
		}
		std::list<Node> successors = FindSuccessors(curNode); // chequea que los 8 vecinos del nodo si no son obstaculos o si se encuentran dentro de mapa
		std::list<Node>::iterator it = successors.begin();
		auto parent = &(close.find(curNode.i * map->GetWidth() + curNode.j)->second);	// puntero. Define al nodo actual como padre para los vecinos.
		while (it != successors.end()) {
			it->parent = parent;
			it->H = ComputeHFromCellToCell(it->i, it->j, goal.i, goal.j);	// calcular heurística
			*it = ResetParent(*it, *it->parent);	// chequea line of sight.
			it->F = it->g + options->hweight * it->H;
			AddOpen(*it);
			it++;
		}
	}

	if (pathfound) {
		MakePrimaryPath(curNode);
	}
	return pathfound;
}


bool ThetaStar::StopCriterion() const {
	if (!openSize) {
		return true;
	}
	return false;
}


void ThetaStar::AddOpen(Node newNode) {
	std::list<Node>::iterator iter, pos;

    // Caso especial: Si la lista 'open' en la posición newNode.i está vacía
	if (open[newNode.i].size() == 0) {	// qué representa la i del nodo??????? será quizá relacionado a posiciones x y en el espacio, pero i=x o i=y?? Si el tamaño de open es height, tonces i debe ser la altura y
		open[newNode.i].push_back(newNode);	// Añade newNode a la lista
		openSize++;	// Incrementa el tamaño de la lista abierta
		return;
	}

	pos = open[newNode.i].end();	// variable para guardar la posición dd se insertará el nuevo nodo, en caso de
	bool posFound = false;	// Variable para indicar si la posición de inserción ha sido encontrada

	// Itera a través de la lista 'open' en la posición newNode.i. Osea, todos los nodos de la misma fila en la lista open revisar nodo que tenga mayor F que el actual. Open estará ordenado?
    for (iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter) {
		// Encuentra la posición donde insertar el nuevo nodo basado en el valor de F (prioridad)
        if (!posFound && iter->F >= newNode.F) {
			// Si F es igual, aplica reglas adicionales para desempate. 
            if (iter->F == newNode.F) {
				if ((options->breakingties == CN_SP_BT_GMAX && newNode.g >= iter->g)
					|| (options->breakingties == CN_SP_BT_GMIN && newNode.g <= iter->g)) {
					pos = iter;
					posFound = true;
				}
			}
			else {	// caso dd se podría dar para insertar
				pos = iter;
				posFound = true;
			}
		}
		// Comprueba si el nodo ya está en la lista abierta y evalúa...
		if (iter->j == newNode.j) {
			// Si el nuevo nodo tiene un F mayor o igual, no se hace nada. El camino hacia ese nodo es peor que el ya existía.
			if (newNode.F >= iter->F)
				return;
			else {	// Si el nuevo nodo tiene un F menor, reemplaza el nodo existente
				// si está en el lugar dd verificamos que debiese estar, reemplazamos
                if (pos == iter) {
					iter->F = newNode.F;
					iter->g = newNode.g;
					iter->parent = newNode.parent;
					return;
				}	// si no está dd verificamos de debíá estar, lo quitamos, xq lo agregaremos dsp en la posición encontrada pos
				open[newNode.i].erase(iter);	// Elimina el nodo existente
				openSize--;	// Decrementa el tamaño de la lista abierta
				break;
			}
		}
	}
	openSize++;	// Incrementa el tamaño de la lista abierta
	open[newNode.i].insert(pos, newNode);	// Inserta el nuevo nodo en la posición encontrada
}


Node ThetaStar::FindMin() {
	Node min;
	min.F = std::numeric_limits<double>::infinity();
	for (int i = 0; i < open.size(); i++) {
		if (!open[i].empty() && open[i].begin()->F <= min.F) {	// open[i] guarda el dato F más pequeño por fila (?)
			if (open[i].begin()->F == min.F) {	// caso de empate
				if ((options->breakingties == CN_SP_BT_GMAX && open[i].begin()->g >= min.g)			// mecanismos de desempate
					|| (options->breakingties == CN_SP_BT_GMIN && open[i].begin()->g <= min.g)) {	// mecanismos de desempate
					min = *open[i].begin();
				}
			}
			else {
				min = *open[i].begin();	// caso en que no hay empate
			}
		}
	}
	return min;
}

// heuristic?SearchPath
float ThetaStar::ComputeHFromCellToCell(int i1, int j1, int i2, int j2) const {
	switch (options->metrictype) {
		case CN_SP_MT_EUCL:
			return static_cast<float>(sqrt((i2 - i1) * (i2 - i1) + (j2 - j1) * (j2 - j1)));				// Se usa esta!! dist euclidiana
		case CN_SP_MT_DIAG:
			return static_cast<float>(abs(abs(i2 - i1) - abs(j2 - j1)) +
									  sqrt(2) * (std::min(abs(i2 - i1), abs(j2 - j1))));
		case CN_SP_MT_MANH:																				// dist manhattan
			return (abs(i2 - i1) + abs(j2 - j1));
		case CN_SP_MT_CHEB:
			return std::max(abs(i2 - i1), abs(j2 - j1));
		default:
			return 0;
	}
}


Node ThetaStar::ResetParent(Node current, Node parent) {

	if (parent.parent == nullptr)
		return current;
	if (current == *parent.parent)
		return current;

	if (visChecker.checkLine(parent.parent->i, parent.parent->j, current.i, current.j, *map)) {		// a lo mjr aquí chequea el line of sight. Asies
		current.g = parent.parent->g + Distance(parent.parent->i, parent.parent->j, current.i, current.j);
		current.parent = parent.parent;
		return current;
	}
	return current;
}


float ThetaStar::Distance(int i1, int j1, int i2, int j2) const {
	return static_cast<float>(sqrt(pow(i1 - i2, 2.0f) + pow(j1 - j2, 2.0f)));
}


void ThetaStar::MakePrimaryPath(Node curNode) {
	// std::cout<< "MakePrimaryPath"<<std::endl;
	Node current = curNode;
	while (current.parent) {
		// std::cout<<map->GetPoint(current).ToString();
		// std::cout<<"\n";	
		currPath.push_front(map->GetPoint(current));
		current = *current.parent;
	}
	// std::cout<<map->GetPoint(current).ToString();	// comentado originalmente
	// std::cout<<map->GetPoint(current).ToString();
	// std::cout<<"\n";	
	// std::cout<<"\n";	
}


std::list<Node> ThetaStar::FindSuccessors(Node curNode) {
	Node newNode;
	std::list<Node> successors;
	for (int i = -1; i <= +1; i++)
		for (int j = -1; j <= +1; j++)
			if ((i != 0 || j != 0) && map->CellOnGrid(curNode.i + i, curNode.j + j) &&	// chequeo a vecinos, no a sí mismo
				(visChecker.checkTraversability(curNode.i + i, curNode.j + j, *map))) {	// chequea que la celda esté en el mapa y no sea un obstáculo
				if (i != 0 && j != 0) {	// solo chequeo diagonales para verificar si puedo desplazarme en ellas
					if (!options->cutcorners) {	// veo si tengo habilitada las esquinas
						if (map->CellIsObstacle(curNode.i, curNode.j + j) ||
							map->CellIsObstacle(curNode.i + i, curNode.j))
							continue;
					}
					else if (!options->allowsqueeze) {	// veo si tengo habilitada las esquinas
						if (map->CellIsObstacle(curNode.i, curNode.j + j) &&
							map->CellIsObstacle(curNode.i + i, curNode.j))
							continue;
					}
				}
				if (close.find((curNode.i + i) * map->GetWidth() + curNode.j + j) == close.end()) {	// si el nodo no está en la lista cerrada
					newNode.i = curNode.i + i;
					newNode.j = curNode.j + j;
					if (i == 0 || j == 0)	// si no soy diagonal
						newNode.g = curNode.g + 1;
					else					// si soy diagonal
						newNode.g = curNode.g + sqrt(2);
					successors.push_front(newNode);
				}
			}
	return successors;
}


bool ThetaStar::CreateGlobalPath() {
	if (!glPathCreated) {	// si no hay un global path creado...
		Node start = map->GetClosestNode(glStart);	// obtiene un nodo del mapa a partir de las coord del xml. Habrá problema acá? No. Acá confirmo que usa coords xr yr. Aún no sé para qué son las otras
		Node goal = map->GetClosestNode(glGoal);

		glPathCreated = SearchPath(start, goal);
		//currPath.push_back(glGoal); 
	}
	return glPathCreated;
}


ThetaStar &ThetaStar::operator=(const ThetaStar &obj) {
	if (this != &obj) {
		PathPlanner::operator=(obj);
		currPath = obj.currPath;
		close = obj.close;
		open = obj.open;
		openSize = obj.openSize;
		glPathCreated = obj.glPathCreated;
		visChecker = obj.visChecker;
		past = obj.past;
	}
	return *this;
}

ThetaStar *ThetaStar::Clone() const {
	return new ThetaStar(*this);
}

void ThetaStar::AddPointToPath(Point p) {
	currPath.push_front(p);
}

Point ThetaStar::PullOutNext() {
	if (currPath.size() > 0) {
		Point res = currPath.front();
		if (currPath.size() > 1) {
			currPath.pop_front();
		}
		return res;
	}

	return Point();
}

Point ThetaStar::GetPastPoint() {
	return past;
}
