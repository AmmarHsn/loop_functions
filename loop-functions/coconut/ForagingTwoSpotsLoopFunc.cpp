/**
  * @file <loop-functions/ForagingTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "ForagingTwoSpotsLoopFunc.h"

/****************************************/
/****************************************/

ForagingTwoSpotsLoopFunction::ForagingTwoSpotsLoopFunction() {
  m_fRadius = 0.15;
  m_fNestLimit = -0.6;
  m_cCoordSpot1 = CVector2(0.75,0);
  m_cCoordSpot2 = CVector2(-0.75,0);
  m_fObjectiveFunction = 0;
  m_bInitializationStep = true;
}

/****************************************/
/****************************************/

ForagingTwoSpotsLoopFunction::ForagingTwoSpotsLoopFunction(const ForagingTwoSpotsLoopFunction& orig) {
}

/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
}

/****************************************/
/****************************************/

ForagingTwoSpotsLoopFunction::~ForagingTwoSpotsLoopFunction() {}

/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::Destroy() {
  delete[] m_punFoodData;
}

/****************************************/
/****************************************/

argos::CColor ForagingTwoSpotsLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  d = (m_cCoordSpot2 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  if (vCurrentPoint.GetY() > m_fNestLimit)
    return CColor::GRAY50;
  else
    return CColor::WHITE;
}


/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::Reset() {
  m_unNumberRobots = 70;
  m_fObjectiveFunction = 0;
  for (UInt32 i = 0; i <= m_unNumberRobots; i++) {
    m_punFoodData[i] = 0;
  }
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::PostStep() {
  ArrestTrespassers();
  m_unNumberRobots = 70;
  if (m_bInitializationStep) {
    m_punFoodData = new UInt32[m_unNumberRobots+1];
    for (UInt32 i = 0; i <= m_unNumberRobots; i++) {
      m_punFoodData[i] = 0;
    }
    m_bInitializationStep = false;
  }
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  UInt32 unId;
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    //unId = atoi(pcEpuck->GetId().substr(8, 2).c_str());
    std::string tmp = pcEpuck->GetId().substr(pcEpuck->GetId().find("k") + 1);
    unId = atoi(tmp.substr(tmp.find("_") + 1).c_str());
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    Real fDistanceSpot2 = (m_cCoordSpot2 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius) {
      m_punFoodData[unId] = 1;
    } else if (fDistanceSpot2 <= m_fRadius){
      m_punFoodData[unId] = 1;
    } else if (cEpuckPosition.GetY() <= m_fNestLimit) {
      m_fObjectiveFunction -= m_punFoodData[unId];
      m_punFoodData[unId] = 0;
    }
  }
  // LOG << "Obj " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real ForagingTwoSpotsLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 ForagingTwoSpotsLoopFunction::GetRandomPosition() {
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  // If b < a, swap them
  if (b < a) {
    temp = a;
    a = b;
    b = temp;
  }
  Real fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
  Real fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));

  return CVector3(fPosX, fPosY, 0);
}

void ForagingTwoSpotsLoopFunction::ArrestTrespassers() {
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

CVector3 ForagingTwoSpotsLoopFunction::GetJailPosition() {
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

    Real fPosX = a*2.5 - 1.250;
    Real fPosY = b*0.5 + 1.950;

    return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(ForagingTwoSpotsLoopFunction, "foraging_two_spots");
