/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "CHOP_CPlusPlusBase.h"

/*

This example file implements a class that does 2 different things depending on
if a CHOP is connected to the CPlusPlus CHOPs input or not.
The example is timesliced, which is the more complex way of working.

If an input is connected the node will output the same number of channels as the
input and divide the first 'N' samples in the input channel by 2. 'N' being the current
timeslice size. This is noteworthy because if the input isn't changing then the output
will look wierd since depending on the timeslice size some number of the first samples
of the input will get used.

If no input is connected then the node will output a smooth sine wave at 120hz.
*/

#include "Box2D/Box2D.h"
#include "SFML/System/Clock.hpp"
//#include <cmath>
//#include <cfloat>
//#include <cstddef>
//#include <limits>

struct Settings
{
	Settings()
	{
		hz = 60.0f;
		velocityIterations = 8;
		positionIterations = 3;
		drawShapes = true;
		drawJoints = true;
		drawAABBs = false;
		drawContactPoints = false;
		drawContactNormals = false;
		drawContactImpulse = false;
		drawFrictionImpulse = false;
		drawCOMs = false;
		drawStats = false;
		drawProfile = false;
		enableWarmStarting = true;
		enableContinuous = true;
		enableSubStepping = false;
		enableSleep = true;
		pause = false;
		singleStep = false;
	}

	float hz;
	int32 velocityIterations;
	int32 positionIterations;
	bool drawShapes;
	bool drawJoints;
	bool drawAABBs;
	bool drawContactPoints;
	bool drawContactNormals;
	bool drawContactImpulse;
	bool drawFrictionImpulse;
	bool drawCOMs;
	bool drawStats;
	bool drawProfile;
	bool enableWarmStarting;
	bool enableContinuous;
	bool enableSubStepping;
	bool enableSleep;
	bool pause;
	bool singleStep;
};


// To get more help about these functions, look at CHOP_CPlusPlusBase.h
class CPlusPlusCHOPExample : public CHOP_CPlusPlusBase
{
public:
	CPlusPlusCHOPExample(const OP_NodeInfo* info);
	virtual ~CPlusPlusCHOPExample();

	virtual void		getGeneralInfo(CHOP_GeneralInfo* ) override;
	virtual bool		getOutputInfo(CHOP_OutputInfo*) override;
	virtual const char*	getChannelName(int32_t index, void* reserved) override;

	virtual void		execute(const CHOP_Output*,
								OP_Inputs*,
								void* reserved) override;


	virtual int32_t		getNumInfoCHOPChans() override;
	virtual void		getInfoCHOPChan(int index,
										OP_InfoCHOPChan* chan) override;

	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize) override;
	virtual void		getInfoDATEntries(int32_t index,
											int32_t nEntries,
											OP_InfoDATEntries* entries) override;

	virtual void		setupParameters(OP_ParameterManager* manager) override;
	virtual void		pulsePressed(const char* name) override;

private:

	// We don't need to store this pointer, but we do for the example.
	// The OP_NodeInfo class store information about the node that's using
	// this instance of the class (like its name).
	const OP_NodeInfo		*myNodeInfo;

	// In this example this value will be incremented each time the execute()
	// function is called, then passes back to the CHOP 
	int32_t					 myExecuteCount;


	double					 myOffset;

	// Box2D stuff
	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	b2Body* m_groundBody;
	b2AABB m_worldAABB;
	//ContactPoint m_points[k_maxContactPoints];
	int32 m_pointCount;
	//DestructionListener m_destructionListener;
	int32 m_textLine;
	b2World* m_world;
	b2Body* m_bomb;
	b2MouseJoint* m_mouseJoint;
	b2Vec2 m_bombSpawnPoint;
	bool m_bombSpawning;
	b2Vec2 m_mouseWorld;
	int32 m_stepCount;

	b2Profile m_maxProfile;
	b2Profile m_totalProfile;

	Settings *settings = new Settings();
	int numCircles = 0;

	bool _mouseDown = false;
	float accumulator;
	float currentTime;
	sf::Clock clock;

	// methods
	void CPlusPlusCHOPExample::createFixedObjects();
	void CPlusPlusCHOPExample::CreateCircle();

	//virtual void CPlusPlusCHOPExample::Keyboard(int key) { B2_NOT_USED(key); }
	//virtual void CPlusPlusCHOPExample::KeyboardUp(int key) { B2_NOT_USED(key); }
	virtual void CPlusPlusCHOPExample::MouseDown(const b2Vec2& p);
	virtual void CPlusPlusCHOPExample::MouseUp(const b2Vec2& p);
	void CPlusPlusCHOPExample::MouseMove(const b2Vec2& p);

};
