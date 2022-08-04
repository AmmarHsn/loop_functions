#include "FastPointFindingLoopFunc.h"

/****************************************/
/****************************************/

FastPointFindingLoopFunction::FastPointFindingLoopFunction() {
  m_fRadius = 0.10;
  m_cCoordSpot1 = CVector2(0.6,0.8);
  m_CCoordRect1Pos = CVector2(0.8,-0.5);

  m_CCoordRect2Pos = CVector2(-0.8,-1);

  m_unState = 0;
  m_unTbar = 0;
  m_fObjectiveFunction = 0;

}

/****************************************/
/****************************************/

FastPointFindingLoopFunction::FastPointFindingLoopFunction(const FastPointFindingLoopFunction& orig) {}

/****************************************/
/****************************************/

FastPointFindingLoopFunction::~FastPointFindingLoopFunction() {}

/****************************************/
/****************************************/

void FastPointFindingLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void FastPointFindingLoopFunction::Reset() {
    CoreLoopFunctions::Reset();
    m_unState = 0;
    m_unTbar = 0;
    m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

void FastPointFindingLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
}

argos::CColor FastPointFindingLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();

  if (d <= m_fRadius) {
    return CColor::WHITE;
  }

  return CColor::GRAY50;
}


CVector3 FastPointFindingLoopFunction::GetRandomPosition() {
    Real a;
    Real b;

    a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

    Real fPosX = m_CCoordRect2Pos.GetX() + a*fabs(m_CCoordRect2Pos.GetX() - m_CCoordRect1Pos.GetX());
    Real fPosY = m_CCoordRect2Pos.GetY() + b*fabs(m_CCoordRect2Pos.GetY() - m_CCoordRect1Pos.GetY());

    return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

void FastPointFindingLoopFunction::PostStep() {
  ArrestTrespassers();
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);

  if (m_unState == 0) {
    m_unTbar +=1;

    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
      CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
      cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                      pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

      Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();

      if (fDistanceSpot1 <= m_fRadius) {
          m_unState = 1;
      }
    }
  }
}

/****************************************/
/****************************************/

void FastPointFindingLoopFunction::PostExperiment() {
  m_fObjectiveFunction = (Real) m_unTbar;
  // LOG << "Final value : "<< m_fObjectiveFunction << std::endl;
}

Real FastPointFindingLoopFunction::GetObjectiveFunction() {
  return (m_fObjectiveFunction);
}

void FastPointFindingLoopFunction::ArrestTrespassers() {
  CEPuckEntity* pcEpuck;
  bool bPlaced = false;
  UInt32 unTrials;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    pcEpuck = any_cast<CEPuckEntity*>(it->second);
    // Choose position at random
    Real posY = pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
    Real posX = pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
    if ( pow(posY, 2.0) + pow(posX, 2.0) > pow(1.250, 2.0) && posY < 1.900 ) {
      unTrials = 0;
      do {
        ++unTrials;
        CVector3 cEpuckPosition = GetJailPosition();
        bPlaced = MoveEntity(pcEpuck->GetEmbodiedEntity(),
                              cEpuckPosition,
                              CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO,CRadians::TWO_PI)),
                              CRadians::ZERO,CRadians::ZERO),false);
      } while(!bPlaced && unTrials < 100);
      if(!bPlaced) {
        THROW_ARGOSEXCEPTION("Can't place robot");
      }
    }
  }
}

CVector3 FastPointFindingLoopFunction::GetJailPosition() {
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

    Real fPosX = a*2.5 - 1.250;
    Real fPosY = b*0.5 + 1.950;

    return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(FastPointFindingLoopFunction, "fast_point_finding");
