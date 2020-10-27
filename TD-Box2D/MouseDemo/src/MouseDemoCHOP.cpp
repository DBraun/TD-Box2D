/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "MouseDemoCHOP.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <assert.h>
#include <algorithm>

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
int32_t
GetCHOPAPIVersion(void)
{
	// Always return CHOP_CPLUSPLUS_API_VERSION in this function.
	return CHOP_CPLUSPLUS_API_VERSION;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new CPlusPlusCHOPExample(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (CPlusPlusCHOPExample*)instance;
}

};

#define	RAND_LIMIT	32767

/// Random number in range [-1,1]
inline float RandomFloat()
{
	float r = (float)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

/// Random floating point number in range [lo, hi]
inline float RandomFloat(float lo, float hi)
{
	float r = (float)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

CPlusPlusCHOPExample::CPlusPlusCHOPExample(const OP_NodeInfo* info) : myNodeInfo(info)
{
	myExecuteCount = 0;
	myOffset = 0.0;

	// Box2D stuff
	b2Vec2 gravity;
	//gravity.Set(0.0f, -9.81f);
	gravity.Set(0.0f, 0.);
	m_world = new b2World(gravity);
	m_bomb = NULL;
	m_textLine = 30;
	m_mouseJoint = NULL;
	m_pointCount = 0;

	//m_destructionListener.test = this;
	//m_world->SetDestructionListener(&m_destructionListener);
	//m_world->SetContactListener(this);
	//m_world->SetDebugDraw(&g_debugDraw);

	m_bombSpawning = false;

	m_stepCount = 0;

	b2BodyDef bodyDef;
	m_groundBody = m_world->CreateBody(&bodyDef);

	memset(&m_maxProfile, 0, sizeof(b2Profile));
	memset(&m_totalProfile, 0, sizeof(b2Profile));

	createFixedObjects();
}

void
CPlusPlusCHOPExample::createFixedObjects() {
	b2BodyDef bd;
	b2Body* ground = m_world->CreateBody(&bd);

	b2EdgeShape shape;

	// Floor
	shape.SetTwoSided(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
	ground->CreateFixture(&shape, 0.0f);

	// Left wall
	shape.SetTwoSided(b2Vec2(-20.0f, 0.0f), b2Vec2(-20.0f, 20.0f));
	ground->CreateFixture(&shape, 0.0f);

	// Right wall
	shape.SetTwoSided(b2Vec2(20.0f, 0.0f), b2Vec2(20.0f, 20.0f));
	ground->CreateFixture(&shape, 0.0f);

	// Roof
	shape.SetTwoSided(b2Vec2(-20.0f, 20.0f), b2Vec2(20.0f, 20.0f));
	ground->CreateFixture(&shape, 0.0f);

	// more stuff
	// todo:

	// Chain shape
	//{
	//	b2BodyDef bd;
	//	bd.angle = 0.25f * b2_pi;
	//	b2Body* ground = m_world->CreateBody(&bd);

	//	b2Vec2 vs[4];
	//	vs[0].Set(5.0f, 7.0f);
	//	vs[1].Set(6.0f, 8.0f);
	//	vs[2].Set(7.0f, 8.0f);
	//	vs[3].Set(8.0f, 7.0f);
	//	b2ChainShape shape;
	//	shape.CreateChain(vs, 4);
	//	ground->CreateFixture(&shape, 0.0f);
	//}
}

class UserData {
	public:
		b2Vec2 previous_position;
		b2Vec2 previous_angle;
};

void
CPlusPlusCHOPExample::CreateCircle() {

	numCircles++;

	float radius = 0.25;
	b2CircleShape shape;
	shape.m_p.SetZero();
	shape.m_radius = radius;

	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = 1.0f;
	fd.friction = 0.8f;

	b2Vec2 p(RandomFloat(-20.f,20.f),  RandomFloat(0.f,20.f));
	//b2Vec2 p(RandomFloat(), 3.0f + RandomFloat());
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position = p;
	UserData *data = new UserData();
	// bd.userData = data; // todo: is this a memory leak?
	bd.userData.pointer = reinterpret_cast<uintptr_t>(data); // todo: is this a memory leak?
	//bd.allowSleep = false;
	b2Body* body = m_world->CreateBody(&bd);
	body->SetLinearDamping(.3f);

	body->CreateFixture(&fd);
}

class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		m_point = point;
		m_fixture = NULL;
	}

	bool ReportFixture(b2Fixture* fixture) override
	{
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody)
		{
			bool inside = fixture->TestPoint(m_point);
			if (inside)
			{
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

void CPlusPlusCHOPExample::MouseDown(const b2Vec2& p)
{
	m_mouseWorld = p;

	if (m_mouseJoint != NULL) {
		return;
	}

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	m_world->QueryAABB(&callback, aabb);

	if (callback.m_fixture)
	{
		b2Body* body = callback.m_fixture->GetBody();
		float frequencyHz = 5.0f;
		float dampingRatio = 0.7f;
		b2MouseJointDef jd;
		jd.bodyA = m_groundBody;
		jd.bodyB = body;
		jd.target = p;
		//jd.dampingRatio = 1.;
		jd.maxForce = 1000.0f * body->GetMass();
		b2LinearStiffness(jd.stiffness, jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
		m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&jd);
		body->SetAwake(true);
	}
}

void CPlusPlusCHOPExample::MouseUp(const b2Vec2& p) {
	if (m_mouseJoint) {
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
	}
}

void CPlusPlusCHOPExample::MouseMove(const b2Vec2& p)
{
	m_mouseWorld = p;

	if (m_mouseJoint) {
		m_mouseJoint->SetTarget(p);
	}
}

CPlusPlusCHOPExample::~CPlusPlusCHOPExample()
{
	delete m_world;
	m_world = NULL;
}

void
CPlusPlusCHOPExample::getGeneralInfo(CHOP_GeneralInfo* ginfo)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->timeslice = false;
	ginfo->inputMatchIndex = 0;
}

bool
CPlusPlusCHOPExample::getOutputInfo(CHOP_OutputInfo* info)
{
	info->numChannels = 6;
	info->sampleRate = 60;
	info->numSamples = numCircles;
	return true;
}

const char*
CPlusPlusCHOPExample::getChannelName(int32_t index, void* reserved) {

	switch (index) {
		case 0:
			return "tx";
			break;
		case 1:
			return "ty";
			break;
		case 2:
			return "vx";
			break;
		case 3:
			return "vy";
			break;
		case 4:
			return "scale";
			break;
		case 5:
			return "id";
			break;
	}

	return "chan1";
}

void
CPlusPlusCHOPExample::execute(const CHOP_Output* output,
							  OP_Inputs* inputs,
							  void* reserved)
{
	myExecuteCount++;

	inputs->enablePar("Mouseforce", 1);
	inputs->enablePar("Reset", 1);
	inputs->enablePar("Shape", 1);
	
	// Box 2D stuff
	float timeStep = settings->hz > 0.0f ? 1.0f / settings->hz : 0.0f;

	if (settings->pause) {
		if (settings->singleStep) {
			settings->singleStep = 0;
		} else {
			timeStep = 0.0f;
		}
	}

	uint32 flags = 0;
	flags += settings->drawShapes			* b2Draw::e_shapeBit;
	flags += settings->drawJoints			* b2Draw::e_jointBit;
	flags += settings->drawAABBs			* b2Draw::e_aabbBit;
	flags += settings->drawCOMs				* b2Draw::e_centerOfMassBit;

	m_world->SetAllowSleeping(settings->enableSleep);
	m_world->SetWarmStarting(settings->enableWarmStarting);
	m_world->SetContinuousPhysics(settings->enableContinuous);
	m_world->SetSubStepping(settings->enableSubStepping);

	m_pointCount = 0;

	// start apply forces from TouchDesigner
	int i = 0;
	if (inputs->getNumInputs() > 0) {
		const OP_CHOPInput	*cinput = inputs->getInputCHOP(0);
		int numSamples = output->numSamples;

		for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
			if (b->GetType() != b2_dynamicBody) {
				continue;
			}
			int index = std::min(i, numSamples - 1);

			float force_x = cinput->getChannelData(0)[index];
			float force_y = cinput->getChannelData(1)[index];
			b->ApplyForce(b2Vec2(force_x, force_y), b->GetWorldCenter(), true);
			i++;
		}
	}
	// end apply force from TouchDesigner

	float deltaTime = clock.getElapsedTime().asSeconds();
	clock.restart();

	const int MAX_STEPS = 5;

	if (accumulator / timeStep > MAX_STEPS) {
		accumulator = 0.0; // avoid spiral of death?
		// This seems necessary because the accumulator may be large at the very first load
		// of the dll in TouchDesigner, so we need to be lenient and reset it.
	}

	accumulator += deltaTime;
	
	i = 0;
	while (accumulator > timeStep && i < MAX_STEPS) {
		accumulator -= timeStep;
		if (accumulator <= timeStep) {
			// Then this is the last step, so reset the state
			for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
				if (b->GetType() != b2_dynamicBody) {
					continue;
				}
				b->GetUserData();
				UserData* data = (UserData*)(b->GetUserData()).pointer;  // todo
				data->previous_position = b->GetPosition();
				//data->previous_angle = b->GetAngle();
			}
		}

		m_world->Step(timeStep, settings->velocityIterations, settings->positionIterations);
		m_world->ClearForces();
		
		i++;
	}

	const float alpha = accumulator / timeStep;
	const float oneMinusAlpha = 1.f - alpha;

	if (timeStep > 0.0f) {
		++m_stepCount;
	}

	if (settings->drawStats)
	{
		int32 bodyCount = m_world->GetBodyCount();
		int32 contactCount = m_world->GetContactCount();
		int32 jointCount = m_world->GetJointCount();
		//g_debugDraw.DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount);
		//m_textLine += DRAW_STRING_NEW_LINE;

		int32 proxyCount = m_world->GetProxyCount();
		int32 height = m_world->GetTreeHeight();
		int32 balance = m_world->GetTreeBalance();
		float quality = m_world->GetTreeQuality();
		//g_debugDraw.DrawString(5, m_textLine, "proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
		//m_textLine += DRAW_STRING_NEW_LINE;
	}

	// Track maximum profile times
	{
		const b2Profile& p = m_world->GetProfile();
		m_maxProfile.step = b2Max(m_maxProfile.step, p.step);
		m_maxProfile.collide = b2Max(m_maxProfile.collide, p.collide);
		m_maxProfile.solve = b2Max(m_maxProfile.solve, p.solve);
		m_maxProfile.solveInit = b2Max(m_maxProfile.solveInit, p.solveInit);
		m_maxProfile.solveVelocity = b2Max(m_maxProfile.solveVelocity, p.solveVelocity);
		m_maxProfile.solvePosition = b2Max(m_maxProfile.solvePosition, p.solvePosition);
		m_maxProfile.solveTOI = b2Max(m_maxProfile.solveTOI, p.solveTOI);
		m_maxProfile.broadphase = b2Max(m_maxProfile.broadphase, p.broadphase);

		m_totalProfile.step += p.step;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.solveInit += p.solveInit;
		m_totalProfile.solveVelocity += p.solveVelocity;
		m_totalProfile.solvePosition += p.solvePosition;
		m_totalProfile.solveTOI += p.solveTOI;
		m_totalProfile.broadphase += p.broadphase;
	}

	// start mouse interactive
	double Mousedown = inputs->getParDouble("Mousedown");
	double Mouseposx;
	double Mouseposy;
	inputs->getParDouble2("Mousepos", Mouseposx, Mouseposy);
	b2Vec2 p;
	p.Set((float)Mouseposx, (float)Mouseposy);

	if (Mousedown) {
		if (_mouseDown) {
			// mouse move
			MouseMove(p);
		}
		else {
			_mouseDown = true;
			// mouse down
			MouseDown(p);
		}
	}
	else {
		if (_mouseDown) {
			_mouseDown = false;
			// mouse up
			MouseUp(p);
		}
	}
	// end mouse interactive

	i = 0;
	for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
		if (b->GetType() != b2_dynamicBody) {
			continue;
		}

		b2Vec2 p = b->GetPosition();
		b2Vec2 v = b->GetLinearVelocity();
		if (p.x <= -10.0f || 10.0f <= p.x || p.y <= 0.0f || 20.0f <= p.y) {
			p.x += 0.0f;
		}

		//  interpolation
	    // https://github.com/just4phil/bubblr3_00.010a/blob/master/bubblr3/src/de/philweb/bubblr3/GameObjectManager.java
		UserData* data = (UserData*)(b->GetUserData()).pointer;  // todo
		b2Vec2 previous_position = data->previous_position;
		b2Vec2 position;
		//b2Vec2 previous_angle = data->previous_angle;
		position.x = p.x*alpha + previous_position.x * oneMinusAlpha;
		position.y = p.y*alpha + previous_position.y * oneMinusAlpha;
		//b->SetAngularVelocity(bodyAngle*alpha + angle * oneMinusAlpha);
		// end interpolation

		output->channels[0][i] = position.x;
		output->channels[1][i] = position.y;
		output->channels[2][i] = v.x;
		output->channels[3][i] = v.y;
		output->channels[4][i] = 1.; // rotation
		output->channels[5][i] = 0.;

		i += 1;
	}
	// end step box2d stuff

}

int32_t
CPlusPlusCHOPExample::getNumInfoCHOPChans()
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP.
	return 4;
}

void
CPlusPlusCHOPExample::getInfoCHOPChan(int32_t index,
										OP_InfoCHOPChan* chan)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.
	
	if (index == 0) {
		chan->name = "anchor_tx";
		if (m_mouseJoint) {
			b2Vec2 p1 = m_mouseJoint->GetAnchorB();
			chan->value = (float)p1.x;
		}
		else {
			chan->value = 0.;
		}			
	} else if (index == 1) {
		chan->name = "anchor_ty";
		if (m_mouseJoint) {
			b2Vec2 p1 = m_mouseJoint->GetAnchorB();
			chan->value = (float)p1.y;
		}
		else {
			chan->value = 0.;
		}
	} else if (index == 2) {
		chan->name = "target_tx";
		if (m_mouseJoint) {
			b2Vec2 p2 = m_mouseJoint->GetTarget();
			chan->value = (float)p2.x;
		}
		else {
			chan->value = 0.;
		}
	} else if (index == 3) {
		chan->name = "target_ty";
		if (m_mouseJoint) {
			b2Vec2 p2 = m_mouseJoint->GetTarget();
			chan->value = (float)p2.y;
		}
		else {
			chan->value = 0.;
		}
	}
}

bool		
CPlusPlusCHOPExample::getInfoDATSize(OP_InfoDATSize* infoSize)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
CPlusPlusCHOPExample::getInfoDATEntries(int32_t index,
										int32_t nEntries,
										OP_InfoDATEntries* entries)
{
	// It's safe to use static buffers here because Touch will make its own
	// copies of the strings immediately after this call returns
	// (so the buffers can be reuse for each column/row)
	static char tempBuffer1[4096];
	static char tempBuffer2[4096];

	if (index == 0)
	{
		// Set the value for the first column
#ifdef WIN32
		strcpy_s(tempBuffer1, "executeCount");
#else // macOS
        strlcpy(tempBuffer1, "executeCount", sizeof(tempBuffer1));
#endif
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
#ifdef WIN32
		sprintf_s(tempBuffer2, "%d", myExecuteCount);
#else // macOS
        snprintf(tempBuffer2, sizeof(tempBuffer2), "%d", myExecuteCount);
#endif
		entries->values[1] = tempBuffer2;
	}

	if (index == 1)
	{
		// Set the value for the first column
#ifdef WIN32
        strcpy_s(tempBuffer1, "offset");
#else // macOS
        strlcpy(tempBuffer1, "offset", sizeof(tempBuffer1));
#endif
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
#ifdef WIN32
        sprintf_s(tempBuffer2, "%g", myOffset);
#else // macOS
        snprintf(tempBuffer2, sizeof(tempBuffer2), "%g", myOffset);
#endif
		entries->values[1] = tempBuffer2;
	}
}

void
CPlusPlusCHOPExample::setupParameters(OP_ParameterManager* manager)
{

	// mouse xy
	{
		OP_NumericParameter	np;

		np.name = "Mousepos";
		np.label = "Mouse Position";
		np.defaultValues[0] = .5;
		np.minSliders[0] = 0.0;
		np.maxSliders[0] = 1.0;
		np.defaultValues[1] = .5;
		np.minSliders[1] = 0.0;
		np.maxSliders[1] = 1.0;

		OP_ParAppendResult res = manager->appendFloat(np,2);
		assert(res == OP_ParAppendResult::Success);
	}

	// mouse down toggle
	{
		OP_NumericParameter	np;

		np.name = "Mousedown";
		np.label = "Mouse Down";
		np.defaultValues[0] = 0.0;

		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Mouse Force
	{
		OP_NumericParameter	np;

		np.name = "Mouseforce";
		np.label = "Mouse Force";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] =  10.0;
		
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// scale
	{
		OP_NumericParameter	np;

		np.name = "Scale";
		np.label = "Scale";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] =  10.0;
		
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// shape
	{
		OP_StringParameter	sp;

		sp.name = "Shape";
		sp.label = "Shape";

		sp.defaultValue = "Sine";

		const char *names[] = { "Sine", "Square", "Ramp" };
		const char *labels[] = { "Sine", "Square", "Ramp" };

		OP_ParAppendResult res = manager->appendMenu(sp, 3, names, labels);
		assert(res == OP_ParAppendResult::Success);
	}

	// pulse
	{
		OP_NumericParameter	np;

		np.name = "Reset";
		np.label = "Reset";
		
		OP_ParAppendResult res = manager->appendPulse(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// pulse create circle
	{
		OP_NumericParameter	np;

		np.name = "Createcircle";
		np.label = "Create Circle";

		OP_ParAppendResult res = manager->appendPulse(np);
		assert(res == OP_ParAppendResult::Success);
	}

}

void 
CPlusPlusCHOPExample::pulsePressed(const char* name)
{
	if (!strcmp(name, "Reset")) {
		int i = 0;
		for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
			if (b->GetType() != b2_dynamicBody) {
				continue;
			}
			m_world->DestroyBody(b);
			i += 1;
		}

		numCircles = 0;

		for (int i = 0; i < 800; i++) {
			CreateCircle();
		}

	}
	else if (!strcmp(name, "Createcircle")) {
		CreateCircle();
	}
}

