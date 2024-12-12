#include "agent.h"

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

		std::vector<int> GetMostConstrainedNeighs();

		int SimMotion(std::vector<int> cRank);

		bool UpdateIntentVelocity();

		Point GetIntentVelocity();


	private:
		float fakeRadius;

		Velocity intentV;

		float maxSpeed;

};


#endif //ORCA_ORCAAGENTWITHCNAV_H
