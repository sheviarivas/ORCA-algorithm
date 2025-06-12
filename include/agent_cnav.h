#include "agent.h"
// #include "mission.h"

#ifndef ORCA_ORCAAGENTWITHCNAV_H
#define ORCA_ORCAAGENTWITHCNAV_H


class agent_cnav : public Agent {

	public:
		agent_cnav();

		agent_cnav(const int &id, const Point &start, const Point &goal, const Map &map,
				   const environment_options &options,
				   AgentParam param);

		agent_cnav(const agent_cnav &obj);

		~agent_cnav();


		agent_cnav *Clone() const override;

		void ComputeNewVelocity() override;

		void ApplyNewVelocity() override;

		bool UpdatePrefVelocity() override;

		bool operator==(const agent_cnav &another) const;

		bool operator!=(const agent_cnav &another) const;

		agent_cnav &operator=(const agent_cnav &obj);


		bool GetMostConstrainedNeighs(std::vector<int> &cRank);

		// int SimMotion(Mission* simulateMission);

		bool UpdateIntentVelocity();

		Point GetIntentVelocity();

		std::vector<std::pair<float, Agent *>> GetNeighbours();

		void SetPrefVelocity(const Velocity &vel);

		Velocity GetPrefVelocity() const;

		float GetMaxSpeed() const;



	private:
		float fakeRadius;

		Velocity intentV;

		float maxSpeed;

		

		// std::vector<int> cRank;

};


#endif //ORCA_ORCAAGENTWITHCNAV_H
