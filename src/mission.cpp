#include "mission.h"


Mission::Mission(std::string fileName, unsigned int agentsNum, unsigned int stepsTh, bool time, size_t timeTh,
				 bool speedStop) {
	taskReader = new XMLReader(fileName);
	agents = vector<Agent *>();
	this->agentsNum = agentsNum;
	stepsTreshhold = stepsTh;
	isTimeBounded = time;
	timeTreshhold = timeTh;

	map = nullptr;
	options = nullptr;
	missionResult = Summary();
	resultsLog = std::unordered_map<int, std::pair<bool, int>>();
	resultsLog.reserve(agentsNum);

#if FULL_LOG
	taskLogger = new XMLLogger(XMLLogger::GenerateLogFileName(fileName, agentsNum), fileName);
	stepsLog = std::unordered_map<int, std::vector<Point>>();
	stepsLog.reserve(agentsNum);

	goalsLog = std::unordered_map<int, std::vector<Point>>();
	goalsLog.reserve(agentsNum);
#endif

	collisionsCount = 0;
	collisionsObstCount = 0;
	stepsCount = 0;

#if MAPF_LOG
	auto found = fileName.find_last_of(".");
	string tmpPAR = fileName.erase(found);
	std::string piece = "_" + std::to_string(agentsNum);
	tmpPAR.insert(found, piece);

	MAPFLog = MAPFInstancesLogger(tmpPAR);
#endif

	commonSpeedsBuffer = std::vector<std::list<float>>(agentsNum, std::list<float>(COMMON_SPEED_BUFF_SIZE, 1.0));
	allStops = false;
	stopByMeanSpeed = speedStop;
}


Mission::Mission(const Mission &obj) {
#if MAPF_LOG
	MAPFLog = obj.MAPFLog;
#endif
	agents = obj.agents;
	map = obj.map;
	options = obj.options;
	missionResult = obj.missionResult;
	resultsLog = obj.resultsLog;
	collisionsCount = obj.collisionsCount;
	collisionsObstCount = obj.collisionsObstCount;
	stepsCount = obj.stepsCount;
	taskReader = (obj.taskReader == nullptr) ? nullptr : obj.taskReader->Clone();


#if FULL_LOG
	taskLogger = (obj.taskLogger == nullptr) ? nullptr : obj.taskLogger->Clone();
	stepsLog = obj.stepsLog;
	goalsLog = obj.goalsLog;

#endif
	commonSpeedsBuffer = obj.commonSpeedsBuffer;
	allStops = obj.allStops;
	stopByMeanSpeed = obj.stopByMeanSpeed;
}


Mission::~Mission() {
	for (auto &agent: agents) {
		if (agent != nullptr) {
			delete agent;
			agent = nullptr;
		}
	}

	if (map != nullptr) {
		delete map;
		map = nullptr;
	}

	if (options != nullptr) {
		delete options;
		options = nullptr;
	}

	if (taskReader != nullptr) {
		delete taskReader;
		taskReader = nullptr;
	}

#if FULL_LOG
	if (taskLogger != nullptr) {
		delete taskLogger;
		taskLogger = nullptr;
	}
#endif
}


bool Mission::ReadTask() {
	// se lee el .xml y se inicializan variables: map, allAgents, options
	return taskReader->ReadData() && taskReader->GetMap(&map) && taskReader->GetAgents(agents, this->agentsNum) &&
		   taskReader->GetEnvironmentOptions(&options);
}


Summary Mission::StartMission() {
#if FULL_OUTPUT
	std::cout << "Start\n";
	std::cout << "\n\n";
#endif
	// creo que se usa para calcular cuanto tiempo toma la ejecución
	auto startpnt = std::chrono::high_resolution_clock::now();
	std::cout << "NumAgents: " << agents.size() << std::endl;
	for (auto agent: agents) {
#if MAPF_LOG
		//was is das???? cmo es pal PARandECBS asi que nnuuuuo
		if (dynamic_cast<ORCAAgentWithPARAndECBS*>(agent) != nullptr) {
			dynamic_cast<ORCAAgentWithPARAndECBS *>(agent)->SetMAPFInstanceLoggerRef(&MAPFLog);
		}
#endif
		// std::cout<<"Nodos Theta Agente "<<agent->GetID()<<std::endl<<std::endl;	// marcado
		bool found = agent->InitPath();
		std:: cout << agent->GetID() << " " << agent->GetPosition().ToString() << " " << agent->GetGoal().ToString() << std::endl;

#if FULL_OUTPUT
		if (!found) {
			
			std::cerr << agent->GetID() << " " << "Path not found\n";
		}
#endif
		resultsLog.insert({agent->GetID(), {false, 0}});

#if FULL_LOG
		stepsLog.insert({agent->GetID(), std::vector<Point>()});
		stepsLog[agent->GetID()].push_back({agent->GetPosition()});
		goalsLog[agent->GetID()].push_back(agent->GetPosition());
#endif

	}

	bool needToStop, needToStopByTime, needToStopBySteps, needToStopBySpeed;




	if (dynamic_cast<agent_cnav*>(agents[0]) != nullptr) {
		std::cout << "hola cnav" << std::endl;
		// inicializar prefV hacia meta
	}
	
	do {
		
		// if (stepsCount > 2) {
		// 	stepsCount = 0;
		// 	for (auto &agent: agents) {
		// 		// dynamic_cast<agent_cnav *>(agent)->
		// 	}
		// 	// agents = backupAgents;
		// 	std::cout<< "get back!!" << std::endl;
		// }
		AssignNeighbours();	// actualiza vecinos (ag. y obs.) añadiendo nuevos o quitando aquellos que se alejaron en el camino

		if ( typeid(*agents[0]) == typeid(agent_cnav)){
			// ask agents most constrained
			std::vector<std::vector<int>> cRanks;	// no muy fan del nombre... pero bue
			for (auto &agent: agents) {
				std::vector<int> cRank;
				dynamic_cast<agent_cnav *>(agent)->GetMostConstrainedNeighs(cRank);
				cRanks.push_back(cRank);
			}

			// debería pasar de otra forma la simulacion? para pensar...
			SimMotion(cRanks);

			
		} else {
			for (auto &agent: agents) {
				agent->UpdatePrefVelocity();	// obtener dirección de vector hacia meta
			}
		}
		

		for (auto &agent: agents) {
			agent->ComputeNewVelocity();	// considerará otros ag., obstáculos, y celdas disponibles contra la dirección obtenida (?)		// acá me parece que termina de funcionar toda la lógica de orca
		}

		UpdateSate();
		auto checkpnt = std::chrono::high_resolution_clock::now();
		size_t nowtime = std::chrono::duration_cast<std::chrono::milliseconds>(checkpnt - startpnt).count();

		needToStopBySpeed = (stopByMeanSpeed and allStops);
		needToStopByTime = (isTimeBounded and nowtime >= timeTreshhold);
		needToStopBySteps = (!isTimeBounded and stepsCount >= stepsTreshhold);
		needToStop = needToStopBySpeed or needToStopByTime or needToStopBySteps;

	} while (!IsFinished() && !needToStop);


	auto endpnt = std::chrono::high_resolution_clock::now();
	size_t res = std::chrono::duration_cast<std::chrono::milliseconds>(endpnt - startpnt).count();

	float stepsSum = 0;
	float rate = 0;
	float MAPFTime = 0.0;
	int initCount = 0, uniCount = 0, updCount = 0, ECBSCount = 0, PARCount = 0, successCount = 0, unsuccessCount = 0, flowtimeMAPF = 0;

	for (auto &node: resultsLog) {

		if (!node.second.first) {
			node.second.second = stepsCount;
		}
		else {
			rate++;
		}
		stepsSum += node.second.second;
	}
	for (auto &agent: agents) {
		collisionsCount += agent->GetCollision().first;
		collisionsObstCount += agent->GetCollision().second;
		auto tmpPARAgent = dynamic_cast<agent_pnr *> (agent);
		if (tmpPARAgent != nullptr) {
			auto statMAPF = tmpPARAgent->GetMAPFStatistics();
			MAPFTime += statMAPF[CNS_MAPF_COMMON_TIME];
			initCount += static_cast<int>(statMAPF[CNS_MAPF_INIT_COUNT]);
			uniCount += static_cast<int>(statMAPF[CNS_MAPF_UNITE_COUNT]);
			updCount += static_cast<int>(statMAPF[CNS_MAPF_UPDATE_COUNT]);
			successCount += static_cast<int>(statMAPF[CNS_MAPF_SUCCESS_COUNT]);
			unsuccessCount += static_cast<int>(statMAPF[CNS_MAPF_UNSUCCESS_COUNT]);
			flowtimeMAPF += static_cast<int>(statMAPF[CNS_MAPF_FLOWTIME]);
		}
		else {
			auto tmpPARnECBSAgent = dynamic_cast<ORCAAgentWithPARAndECBS *> (agent);
			if (tmpPARnECBSAgent != nullptr) {
				//tmpPARnECBSAgent->PrintMAPFMemberStat();
				auto statMAPF = tmpPARnECBSAgent->GetMAPFStatistics();
				MAPFTime += statMAPF[CNS_MAPF_COMMON_TIME];
				initCount += static_cast<int>(statMAPF[CNS_MAPF_INIT_COUNT]);
				uniCount += static_cast<int>(statMAPF[CNS_MAPF_UNITE_COUNT]);
				updCount += static_cast<int>(statMAPF[CNS_MAPF_UPDATE_COUNT]);
				ECBSCount += static_cast<int>(statMAPF[CNS_MAPF_ECBS_COUNT]);
				PARCount += static_cast<int>(statMAPF[CNS_MAPF_PAR_COUNT]);
				successCount += static_cast<int>(statMAPF[CNS_MAPF_SUCCESS_COUNT]);
				unsuccessCount += static_cast<int>(statMAPF[CNS_MAPF_UNSUCCESS_COUNT]);
				flowtimeMAPF += static_cast<int>(statMAPF[CNS_MAPF_FLOWTIME]);
			}
		}
	}

	missionResult[CNS_SUM_SUCCESS_RATE] = std::to_string(rate * 100 / agentsNum);
	missionResult[CNS_SUM_RUN_TIME] = std::to_string(((float) res) / 1000);
	missionResult[CNS_SUM_COLLISIONS] = std::to_string(collisionsCount / 2);
	missionResult[CNS_SUM_FLOW_TIME] = std::to_string(stepsSum * options->timestep);
	missionResult[CNS_SUM_MAKESPAN] = std::to_string(stepsCount * options->timestep);
	missionResult[CNS_SUM_COLLISIONS_OBS] = std::to_string(collisionsObstCount);

	missionResult[CNS_SUM_MAPF_MEAN_TIME] = std::to_string(MAPFTime);
	missionResult[CNS_SUM_MAPF_INIT_COUNT] = std::to_string(initCount);
	missionResult[CNS_SUM_MAPF_UNITE_COUNT] = std::to_string(uniCount);
	missionResult[CNS_SUM_MAPF_UPDATE_COUNT] = std::to_string(updCount);

	missionResult[CNS_SUM_MAPF_ECBS_COUNT] = std::to_string(ECBSCount);
	missionResult[CNS_SUM_MAPF_PAR_COUNT] = std::to_string(PARCount);

	missionResult[CNS_SUM_MAPF_FLOWTIME] = std::to_string(flowtimeMAPF);
	missionResult[CNS_SUM_MAPF_SUCCESS_COUNT] = std::to_string(successCount);
	missionResult[CNS_SUM_MAPF_UNSUCCESS_COUNT] = std::to_string(unsuccessCount);


#if FULL_OUTPUT
	std::cout << "End\n";
#endif
	return missionResult;
}


#if FULL_LOG

bool Mission::SaveLog() {
	taskLogger->SetResults(stepsLog, goalsLog, resultsLog);
	taskLogger->SetSummary(missionResult);
	return taskLogger->GenerateLog() && (stepsCount > 0);
}

#endif


void Mission::UpdateSate() {
	size_t i = 0;
	allStops = true;
	bool print_positions = true;
	if (print_positions) ::cout<< "Step "<< stepsCount<< std::endl;
	
	for (auto &agent: agents) {
		agent->ApplyNewVelocity();	// actualiza currV y meanSavedSpeed. Otros parámetros quedan en false
		Point newPos = agent->GetPosition() + (agent->GetVelocity() * options->timestep);
		agent->SetPosition(newPos);
		commonSpeedsBuffer[i].pop_front();
		commonSpeedsBuffer[i].push_back(agent->GetVelocity().EuclideanNorm());

		float sum = 0.0f;
		float c = 0.0f;
		float y, t;
		float mean;
		for (auto speed: commonSpeedsBuffer[i]) {
			y = speed - c;
			t = sum + y;
			c = (t - sum) - y;
			sum = t;
		}
		mean = sum / commonSpeedsBuffer[i].size();

		if (mean >= MISSION_SMALL_SPEED) {	// qué serás tú...	// quizá para detectar cuando los ag no se estén moviendo	// asies, y si queda true, se mata la simulación
			allStops = false;
		}

#if FULL_LOG
		stepsLog[agent->GetID()].push_back(newPos);
		goalsLog[agent->GetID()].push_back(agent->GetNext());
#endif
		i++;
		if (print_positions) std::cout << agent->GetPosition().ToString()<< std::endl;
		
		// std::cout<< "Posición Ag " << agent->GetID()<< ": " << agent->GetPosition().ToString()<< std::endl;
	}
	// if (print_positions) std::cout<< std::endl;

	stepsCount++;
}


void Mission::AssignNeighbours() {
	for (auto &agent: agents) {

		for (auto &neighbour: agents) {
			if (agent != neighbour) {	// si no soy yo mismo
				float distSq = (agent->GetPosition() - neighbour->GetPosition()).SquaredEuclideanNorm(); // (xa - xb) pow itself, same with y, then add them up
				agent->AddNeighbour(*neighbour, distSq);	// evaluamos si se añade o no a partir de su distancia
			}
		}
		agent->UpdateNeighbourObst();
	}
}


bool Mission::IsFinished() {
	bool result = true;
	for (auto &agent: agents) {
		bool localres = agent->isFinished();
		resultsLog[agent->GetID()].first = agent->isFinished() && resultsLog[agent->GetID()].first;
		if (localres && !resultsLog[agent->GetID()].first) {
			resultsLog[agent->GetID()].first = true;
			resultsLog[agent->GetID()].second = stepsCount;
		}
		result = result && localres;
	}

	return result;
}

Mission &Mission::operator=(const Mission &obj) {
	if (this != &obj) {
		stepsCount = obj.stepsCount;
		stepsTreshhold = obj.stepsTreshhold;
		agentsNum = obj.agentsNum;
		collisionsCount = obj.collisionsCount;
		collisionsObstCount = obj.collisionsObstCount;
		taskReader = obj.taskReader;
		missionResult = obj.missionResult;
		resultsLog = obj.resultsLog;
#if MAPF_LOG
		MAPFLog = obj.MAPFLog;
#endif

		vector<Agent *> tmpAgents = vector<Agent *>(obj.agents.size());
		for (int i = 0; i < obj.agents.size(); i++) {
			tmpAgents.push_back(obj.agents[i]->Clone());
		}

		for (auto &agent: agents) {
			delete agent;
		}

		agents = tmpAgents;

		if (map != nullptr) {
			delete map;
		}
		map = (obj.map == nullptr) ? nullptr : new Map(*obj.map);

		if (options != nullptr) {
			delete options;
		}
		options = (obj.options == nullptr) ? nullptr : new environment_options(*obj.options);

		if (taskReader != nullptr) {
			delete taskReader;
		}
		taskReader = (obj.taskReader == nullptr) ? nullptr : obj.taskReader->Clone();

#if FULL_LOG
		if (taskLogger != nullptr) {
			delete taskLogger;
		}
		taskLogger = (obj.taskLogger == nullptr) ? nullptr : obj.taskLogger->Clone();
		stepsLog = obj.stepsLog;
		goalsLog = obj.goalsLog;
#endif
		commonSpeedsBuffer = obj.commonSpeedsBuffer;
		allStops = obj.allStops;
		stopByMeanSpeed = obj.stopByMeanSpeed;
	}
	return *this;
}

std::unique_ptr<Mission> Mission::Clone() const {
	// return new Mission(*this);
	return std::make_unique<Mission>(*this);
}


bool Mission::SimMotion(std::vector<std::vector<int>> &cRanks){
	// to do: entregar valores correctos de los parámetros k y T
	int k = -1;
	int T = -1;

	// guardar valor Ra de cada ag
	std::vector<int> Ra;

	// clonar agentes
	auto backupAgents = std::make_unique< vector<Agent *> >(agents);

	// por cada ag
	for (int i =0; i<this->agents.size(); ++i){

		// std::cout<< agents[i]->GetID() << std::endl;	

		std::vector<Velocity> actions;
		// obtener prefV
		agent_cnav* currOriginal = dynamic_cast<agent_cnav*>(agents[i]);
		actions.push_back(currOriginal->GetPrefVelocity());

		// aplicar grados beta y demás al resto de acciones
		actions.push_back(Point(0,0));
		actions.push_back(Point(0,0));
		actions.push_back(Point(0,0));
		actions.push_back(Point(0,0));

		// guardar maxRa por cada accion
		std::vector<std::pair<Velocity, float>> maxRa;	// acción y score

		// por cada acción
		for (auto &action: actions) {
			float R_ca = 0;
			float R_ga = 0;

			// clonar Mission o Agentes
			auto simulateTask = this->Clone();
			agent_cnav* currSimulated = dynamic_cast<agent_cnav*>(simulateTask->agents[i]);
			// std::cout<< currSimulated->GetID() << std::endl;

			// podría llamar GetMostConstrained desde aquí

			for (int t=0; t<T; ++t){
				// simular ambiente a curr y vecinos
				std::vector<std::pair<float, Agent *>> neighborhood = {std::make_pair(0, currSimulated)};
				std::vector<std::pair<float, Agent *>> neighbours = currSimulated->GetNeighbours();
				neighborhood.insert(neighborhood.end(), neighbours.begin(), neighbours.end());

				for (auto &neigh: neighborhood) {
					neigh.second->UpdatePrefVelocity();
					neigh.second->ComputeNewVelocity();
					simulateTask->UpdateSate();
				}

				std::vector<int> currSimulatedcRank = cRanks[i];
				// o podria obtener cRank[i] aquí

				if (t>0){
					for(int j=0; j<k; ++j){
						/*
							necesito castear para obtener el cRank
							pero necesito el cRank para poder castear
						*/
						agent_cnav* neigh_j = dynamic_cast<agent_cnav*>(neighbours[currSimulatedcRank[j]].second);
						float maxV_j = neigh_j->GetMaxSpeed();
						float intentV_j = neigh_j->GetIntentVelocity().EuclideanNorm();
						float newV_j = neigh_j->GetVelocity().EuclideanNorm();

						R_ca = R_ca + maxV_j - abs( intentV_j - newV_j );
					}



				}
			}




		}


	}



	// for (int t=0; t<T; ++t){

	// 	for (auto &agent: simulateTask->agents) {
	// 		// debería usar una copia del agente actual... ?
	// 		agent_cnav* curr = dynamic_cast<agent_cnav*>(agent);
	// 		curr->UpdatePrefVelocity();	// obtener dirección de vector hacia meta
	// 		// tonces aquí debería crear la copia de la simulación
	// 		// luego

	// 		// aplicar acción a simulación cambiando prefV
	// 		// me falta aplicarle los angulos pero pa dsp con los ángulos beta
	// 		std::vector<Velocity> actions;
	// 		// for... 
	// 		for (auto &action: actions) {

	// 		}
	// 		curr->SetPrefVelocity(action);

	// 		// luego aplicarle a la simulación a todos
	// 		std::vector<std::pair<float, Agent *>> neighborhood = {std::make_pair(0, curr)};
	// 		std::vector<std::pair<float, Agent *>> neighbours = curr->GetNeighbours();
	// 		neighborhood.insert(neighborhood.end(), neighbours.begin(), neighbours.end());

	// 		for (auto &neigh: neighborhood) {
	// 			neigh.second->UpdatePrefVelocity();
	// 			neigh.second->ComputeNewVelocity();
	// 			simulateTask->UpdateSate();
	// 		}

	// 	}

	// 	// iterar por cada ag luego de la sim para obtener la nueva data

	// 	if (t>0){
	// 		for(int j=0; j<k; ++j){
	// 			agent_cnav* neigh_j = dynamic_cast<agent_cnav*>(Neighbours[cRank[j]].second);
	// 			float maxV_j = neigh_j->maxSpeed;
	// 			float intentV_j = neigh_j->GetIntentVelocity().EuclideanNorm();
	// 			float newV_j = neigh_j->GetVelocity().EuclideanNorm();

	// 			R_ca = R_ca + maxV_j - abs( intentV_j - newV_j );
	// 		}
	// 	}
	// 	float newV_i = this->GetVelocity().EuclideanNorm();


	// 	R_ga = R_ga; // + currV_i * posActual - meta / magnitud
	// }

	// R_ga;
	// R_ga;
	// delete(simulateTask);

	return -1;
}

void Mission::UpdateSimulation(){
	
}