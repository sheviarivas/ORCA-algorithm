#include <set>
#include <algorithm>
#include <chrono>

#include "agent.h"
#include "sub_map.h"
#include "mapf/mapf_actor_set.h"
#include "mapf/mapf_actor.h"
#include "mapf/push_and_rotate.h"
#include "mapf/astar.h"
#include "mapf_instances_logger.h"

#ifndef ORCA_ORCAAGENTWITHPAR_H
#define ORCA_ORCAAGENTWITHPAR_H


class agent_pnr : public Agent {

	public:
		agent_pnr();

		agent_pnr(const int &id, const Point &start, const Point &goal, const Map &map,
				  const environment_options &options,
				  AgentParam param);

		agent_pnr(const agent_pnr &obj);

		~agent_pnr();


		agent_pnr *Clone() const override;

		void ComputeNewVelocity() override;

		void ApplyNewVelocity() override;

		bool UpdatePrefVelocity() override;

		void AddNeighbour(Agent &neighbour, float distSq) override;

		bool isPARMember() const;

		unordered_map<std::string, float> GetMAPFStatistics() const;


		bool operator==(const agent_pnr &another) const;

		bool operator!=(const agent_pnr &another) const;

		agent_pnr &operator=(const agent_pnr &obj);

#if MAPF_LOG

		void SetMAPFInstanceLoggerRef(MAPFInstancesLogger *log);

#endif

	private:
		std::vector<std::pair<float, Agent *>> &GetNeighbours();

		std::set<agent_pnr *> GetAgentsForCentralizedPlanning();

		void SetAgentsForCentralizedPlanning(std::set<agent_pnr *> agents);

		void PreparePARExecution();

		bool ComputePAREnv();

		Point GetGoalPointForMAPF(SubMap Area);

		bool ComputePAR();

		bool UnitePAR();

		bool UpdatePAR();


		SubMap PARMap;		// submapa de PAR inicializado en ComputePAREnv()
		MAPFActorSet PARSet;	// uh??? init en ComputePAREnv()
		MAPFConfig conf;	// config usada en ComputePAR
		std::set<agent_pnr *> PARAgents;	// ags de un PAR
		float fakeRadius;	// uh???
		bool inPARMode;	// uh???	// true al ejecutar PreparePARExecution()
		bool moveToPARPos;		// true al ejecutar PreparePARExecution()
		bool PARVis;	// uh???
		bool notPARVis;
		bool PARUnion;
		bool PARExec;
		bool waitForStart;
		bool waitForFinish;
		Point PARStart;
		Point PARGoal;
		Astar<> PARsearch;
		PushAndRotate PARSolver;
		MAPFSearchResult PARres;
		int currPARPos;		// ?? init en ComputePAREnv()
		int PARActorId;		// 
		std::vector<Point> buffPar;	// roba metas del planner válidas para el submapa del PAR. Inicializado en GetGoalPointForMAPF()

		int initCount;
		int updCount;
		int uniCount;
		float timeMAPF;

		int successCount;
		int unsuccessCount;
		int flowtimeCount;

#if MAPF_LOG
		MAPFInstancesLogger *PARLog;
#endif


};


#endif //ORCA_ORCAAGENTWITHPAR_H
