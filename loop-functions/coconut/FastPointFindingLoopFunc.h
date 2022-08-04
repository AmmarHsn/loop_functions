#ifndef FAST_POINT_FINDING_LOOP_FUNC
#define FAST_POINT_FINDING_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class FastPointFindingLoopFunction: public CoreLoopFunctions {
  public:
    FastPointFindingLoopFunction();
    FastPointFindingLoopFunction(const FastPointFindingLoopFunction& orig);
    virtual ~FastPointFindingLoopFunction();

    virtual void Destroy();

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual CVector3 GetRandomPosition();

    Real GetObjectiveFunction();

    void ArrestTrespassers();
    CVector3 GetJailPosition();

  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot1;
    CVector2 m_CCoordRect2Pos;
    CVector2 m_CCoordRect1Pos;

    typedef std::map<CEPuckEntity*, CVector2 > TOldPosMap;
    TOldPosMap m_tOldPosPoints;

    UInt8 m_unState;
    UInt32 m_unTbar;
    Real m_fObjectiveFunction;
};

#endif
