/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#include "Arbiter.h"
#include "Body.h"
#include <utility>

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

enum Axis
{
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y
};

enum EdgeNumbers
{
	NO_EDGE = 0,
	EDGE1,
	EDGE2,
	EDGE3,
	EDGE4
};

struct ClipVertex
{
	ClipVertex() { fp.value = 0; }
	Vec2 v;
	FeaturePair fp;
};

Vec2 OrthogonalVec(const Vec2& v)
{
	return Vec2(-v.y, v.x);
}

void Flip(FeaturePair& fp)
{
	Swap(fp.e.inEdge1, fp.e.inEdge2);
	Swap(fp.e.outEdge1, fp.e.outEdge2);
}

int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
	const Vec2& normal, float offset, char clipEdge)
{
	int numOut = 0;


	float distance0 = Dot(normal, vIn[0].v) - offset;
	float distance1 = Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > 0.0f)
		{
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.e.inEdge1 = clipEdge;
			vOut[numOut].fp.e.inEdge2 = NO_EDGE;
		}
		else
		{
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.e.outEdge1 = clipEdge;
			vOut[numOut].fp.e.outEdge2 = NO_EDGE;
		}
		++numOut;
	}

	return numOut;
}

static void ComputeIncidentEdge(ClipVertex c[2], const Vec2& h, const Vec2& pos,
	const Mat22& Rot, const Vec2& normal)
{
	// The normal is from the reference box. Convert it
	// to the incident boxe's frame and flip sign.
	Mat22 RotT = Rot.Transpose();
	Vec2 n = -(RotT * normal);
	Vec2 nAbs = Abs(n);

	if (nAbs.x > nAbs.y)
	{
		if (Sign(n.x) > 0.0f)
		{
			c[0].v.Set(h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE3;
			c[0].fp.e.outEdge2 = EDGE4;

			c[1].v.Set(h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE4;
			c[1].fp.e.outEdge2 = EDGE1;
		}
		else
		{
			c[0].v.Set(-h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE1;
			c[0].fp.e.outEdge2 = EDGE2;

			c[1].v.Set(-h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE2;
			c[1].fp.e.outEdge2 = EDGE3;
		}
	}
	else
	{
		if (Sign(n.y) > 0.0f)
		{
			c[0].v.Set(h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE4;
			c[0].fp.e.outEdge2 = EDGE1;

			c[1].v.Set(-h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE1;
			c[1].fp.e.outEdge2 = EDGE2;
		}
		else
		{
			c[0].v.Set(-h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE2;
			c[0].fp.e.outEdge2 = EDGE3;

			c[1].v.Set(h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE3;
			c[1].fp.e.outEdge2 = EDGE4;
		}
	}

	c[0].v = pos + Rot * c[0].v;
	c[1].v = pos + Rot * c[1].v;
}

int BoxToCircle(Body*& bodyB, Body*& bodyA, Contact* contacts)
{
	if (bodyB->shape == 1)
		std::swap(bodyA, bodyB);

	Vec2 circlePos = bodyA->position;
	float radius = bodyA->radius;

	Vec2 boxPos = bodyB->position;
	Vec2 h = 0.5f * bodyB->width;
	Mat22 Rot(bodyB->rotation);
	Mat22 RotT = Rot.Transpose();

	Vec2 localCirclePos = RotT * (circlePos - boxPos);

	Vec2 closestPoint = localCirclePos;
	closestPoint.x = std::max(-h.x, std::min(closestPoint.x, h.x));
	closestPoint.y = std::max(-h.y, std::min(closestPoint.y, h.y));

	Vec2 d = localCirclePos - closestPoint;
	float distSquared = Dot(d, d);


	if (distSquared <= radius * radius)
	{
		float dist = sqrt(distSquared);

		Vec2 normal = dist > 0.0f ? (Rot * d) / dist : Vec2(1.0f, 0.0f);
		normal *= radius;

		contacts[0].position = circlePos - normal;
		contacts[0].normal = normal;
		contacts[0].separation = dist - radius;
		return 1;
	}
	return 0;
}

int BoxToBox(Body* bodyA, Body* bodyB, Contact* contacts)
{
	// Setup
	Vec2 hA = 0.5f * bodyA->width;
	Vec2 hB = 0.5f * bodyB->width;

	Vec2 posA = bodyA->position;
	Vec2 posB = bodyB->position;

	Mat22 RotA(bodyA->rotation), RotB(bodyB->rotation);

	Mat22 RotAT = RotA.Transpose();
	Mat22 RotBT = RotB.Transpose();

	Vec2 dp = posB - posA;
	Vec2 dA = RotAT * dp;
	Vec2 dB = RotBT * dp;

	Mat22 C = RotAT * RotB;
	Mat22 absC = Abs(C);
	Mat22 absCT = absC.Transpose();

	// Box A faces
	Vec2 faceA = Abs(dA) - hA - absC * hB;
	if (faceA.x > 0.0f || faceA.y > 0.0f)
		return 0;

	// Box B faces
	Vec2 faceB = Abs(dB) - absCT * hA - hB;
	if (faceB.x > 0.0f || faceB.y > 0.0f)
		return 0;

	// Find best axis
	Axis axis;
	float separation;
	Vec2 normal;

	// Box A faces
	axis = FACE_A_X;
	separation = faceA.x;
	normal = dA.x > 0.0f ? RotA.col1 : -RotA.col1;

	const float relativeTol = 0.95f;
	const float absoluteTol = 0.01f;

	if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
	{
		axis = FACE_A_Y;
		separation = faceA.y;
		normal = dA.y > 0.0f ? RotA.col2 : -RotA.col2;
	}

	// Box B faces
	if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
	{
		axis = FACE_B_X;
		separation = faceB.x;
		normal = dB.x > 0.0f ? RotB.col1 : -RotB.col1;
	}

	if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
	{
		axis = FACE_B_Y;
		separation = faceB.y;
		normal = dB.y > 0.0f ? RotB.col2 : -RotB.col2;
	}

	// Setup clipping plane data based on the separating axis
	Vec2 frontNormal, sideNormal;
	ClipVertex incidentEdge[2];
	float front, negSide, posSide;
	char negEdge, posEdge;

	// Compute the clipping lines and the line segment to be clipped.
	switch (axis)
	{
	case FACE_A_X:
	{
		frontNormal = normal;
		front = Dot(posA, frontNormal) + hA.x;
		sideNormal = RotA.col2;
		float side = Dot(posA, sideNormal);
		negSide = -side + hA.y;
		posSide = side + hA.y;
		negEdge = EDGE3;
		posEdge = EDGE1;
		ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
	}
	break;

	case FACE_A_Y:
	{
		frontNormal = normal;
		front = Dot(posA, frontNormal) + hA.y;
		sideNormal = RotA.col1;
		float side = Dot(posA, sideNormal);
		negSide = -side + hA.x;
		posSide = side + hA.x;
		negEdge = EDGE2;
		posEdge = EDGE4;
		ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
	}
	break;

	case FACE_B_X:
	{
		frontNormal = -normal;
		front = Dot(posB, frontNormal) + hB.x;
		sideNormal = RotB.col2;
		float side = Dot(posB, sideNormal);
		negSide = -side + hB.y;
		posSide = side + hB.y;
		negEdge = EDGE3;
		posEdge = EDGE1;
		ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
	}
	break;

	case FACE_B_Y:
	{
		frontNormal = -normal;
		front = Dot(posB, frontNormal) + hB.y;
		sideNormal = RotB.col1;
		float side = Dot(posB, sideNormal);
		negSide = -side + hB.x;
		posSide = side + hB.x;
		negEdge = EDGE2;
		posEdge = EDGE4;
		ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
	}
	break;
	}

	// clip other face with 5 box planes (1 face plane, 4 edge planes)

	ClipVertex clipPoints1[2];
	ClipVertex clipPoints2[2];
	int np;

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

	if (np < 2)
		return 0;

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

	if (np < 2)
		return 0;

	// Now clipPoints2 contains the clipping points.
	// Due to roundoff, it is possible that clipping removes all points.

	int numContacts = 0;
	for (int i = 0; i < 2; ++i)
	{
		float separation = Dot(frontNormal, clipPoints2[i].v) - front;

		if (separation <= 0)
		{
			contacts[numContacts].separation = separation;
			contacts[numContacts].normal = normal;
			// slide contact point onto reference face (easy to cull)
			contacts[numContacts].position = clipPoints2[i].v - separation * frontNormal;
			contacts[numContacts].feature = clipPoints2[i].fp;
			if (axis == FACE_B_X || axis == FACE_B_Y)
				Flip(contacts[numContacts].feature);
			++numContacts;
		}
	}
	return numContacts;
}

int TriangleToTriangle(Body* bodyA, Body* bodyB, Contact* contacts)
{
	
	Vec2 vertsA[3] = {
		bodyA->position + Vec2(-bodyA->width.x / 2, -bodyA->width.y / 2),
		bodyA->position + Vec2(bodyA->width.x / 2, -bodyA->width.y / 2),
		bodyA->position + Vec2(0, bodyA->width.y / 2)
	};

	Vec2 vertsB[3] = {
		bodyB->position + Vec2(-bodyB->width.x / 2, -bodyB->width.y / 2),
		bodyB->position + Vec2(bodyB->width.x / 2, -bodyB->width.y / 2),
		bodyB->position + Vec2(0, bodyB->width.y / 2)
	};


	Vec2 axes[6] = {
		OrthogonalVec(vertsA[1] - vertsA[0]).Normalize(),
		OrthogonalVec(vertsA[2] - vertsA[1]).Normalize(),
		OrthogonalVec(vertsA[0] - vertsA[2]).Normalize(),
		OrthogonalVec(vertsB[1] - vertsB[0]).Normalize(),
		OrthogonalVec(vertsB[2] - vertsB[1]).Normalize(),
	    OrthogonalVec(vertsB[0] - vertsB[2]).Normalize()
	};

	float minOverlap = FLT_MAX;
	Vec2 smallestAxis;

	const float epsilon = 1e-6f;
	for (int i = 0; i < 6; ++i)
	{
		float minA = FLT_MAX, maxA = -FLT_MAX;
		float minB = FLT_MAX, maxB = -FLT_MAX;

		for (int j = 0; j < 3; ++j)
		{
			float proj = Dot(vertsA[j], axes[i]);
			minA = std::min(minA, proj);
			maxA = std::max(maxA, proj);
		}

		for (int j = 0; j < 3; ++j)
		{
			float proj = Dot(vertsB[j], axes[i]);
			minB = std::min(minB, proj);
			maxB = std::max(maxB, proj);
		}


		if (maxA < minB - epsilon || maxB < minA - epsilon)
			return 0; 


		float overlap = std::min(maxA, maxB) - std::max(minA, minB);
		if (overlap < minOverlap)
		{
			minOverlap = overlap;
			smallestAxis = axes[i];
		}
	}


	if (Dot(smallestAxis, bodyB->position - bodyA->position) < 0)
		smallestAxis = -smallestAxis;

	contacts[0].separation = -minOverlap;
	contacts[0].normal = smallestAxis;
	contacts[0].position = (vertsA[0] + vertsA[1] + vertsA[2] + vertsB[0] + vertsB[1] + vertsB[2]) / 6.0f;

	return 1; 
}

int BoxToTriangle(Body* bodyA, Body* bodyB, Contact* contacts)
{
	if (bodyA->shape == 0)
		std::swap(bodyA, bodyB);

	Vec2 triVerts[3] = {
		bodyA->position + Vec2(-bodyA->width.x / 2, -bodyA->width.y / 2),
		bodyA->position + Vec2(bodyA->width.x / 2, -bodyA->width.y / 2),
		bodyA->position + Vec2(0, bodyA->width.y / 2)
	};

	Vec2 boxVerts[4] = {
		bodyB->position + Vec2(-bodyB->width.x / 2, -bodyB->width.y / 2),
		bodyB->position + Vec2(bodyB->width.x / 2, -bodyB->width.y / 2),
		bodyB->position + Vec2(bodyB->width.x / 2, bodyB->width.y / 2),
		bodyB->position + Vec2(-bodyB->width.x / 2, bodyB->width.y / 2)
	};

	Vec2 axes[5] = {
		OrthogonalVec(triVerts[1] - triVerts[0]).Normalize(),
		OrthogonalVec(triVerts[2] - triVerts[1]).Normalize(),
		OrthogonalVec(triVerts[0] - triVerts[2]).Normalize(),
		Vec2(1, 0), // Box X-axis
		Vec2(0, 1)  // Box Y-axis
	};


	float minOverlap = FLT_MAX;
	Vec2 smallestAxis;

	for (int i = 0; i < 5; ++i)
	{
		float minTri = FLT_MAX, maxTri = -FLT_MAX;
		float minBox = FLT_MAX, maxBox = -FLT_MAX;

	
		for (int j = 0; j < 3; ++j)
		{
			float proj = Dot(triVerts[j], axes[i]);
			minTri = std::min(minTri, proj);
			maxTri = std::max(maxTri, proj);
		}

		for (int j = 0; j < 4; ++j)
		{
			float proj = Dot(boxVerts[j], axes[i]);
			minBox = std::min(minBox, proj);
			maxBox = std::max(maxBox, proj);
		}

		if (maxTri < minBox || maxBox < minTri)
			return 0; 

		float overlap = std::min(maxTri, maxBox) - std::max(minTri, minBox);
		if (overlap < minOverlap)
		{
			minOverlap = overlap;
			smallestAxis = axes[i];
		}
	}
	contacts[0].separation = -minOverlap;
	contacts[0].normal = smallestAxis;
	contacts[0].position = (triVerts[0] + triVerts[1] + triVerts[2]) / 3;
	return 1; 
}

int CircleToTriangle(Body* bodyA, Body* bodyB, Contact* contacts)
{
	if (bodyA->shape == 2)
		std::swap(bodyA, bodyB);

	Vec2 circlePos = bodyA->position;
	float radius = bodyA->radius;

	Vec2 triangleVerts[3] = {
		bodyB->position + Vec2(-bodyB->width.x / 2, -bodyB->width.y / 2),
		bodyB->position + Vec2(bodyB->width.x / 2, -bodyB->width.y / 2),
		bodyB->position + Vec2(0, bodyB->width.y / 2)
	};


	Vec2 edge1 = triangleVerts[1] - triangleVerts[0];
	Vec2 edge2 = triangleVerts[2] - triangleVerts[1];
	Vec2 edge3 = triangleVerts[0] - triangleVerts[2];

	Vec2 toCircle1 = circlePos - triangleVerts[0];
	Vec2 toCircle2 = circlePos - triangleVerts[1];
	Vec2 toCircle3 = circlePos - triangleVerts[2];

	float cross1 = Cross(edge1, toCircle1);
	float cross2 = Cross(edge2, toCircle2);
	float cross3 = Cross(edge3, toCircle3);

	bool isInside = (cross1 >= 0 && cross2 >= 0 && cross3 >= 0) || (cross1 <= 0 && cross2 <= 0 && cross3 <= 0);

	if (isInside)
	{
		contacts[0].position = circlePos;
		contacts[0].normal = Vec2(0, 1); 
		contacts[0].separation = -radius;
		return 1;
	}

	for (int i = 0; i < 3; ++i)
	{
		Vec2 p1 = triangleVerts[i];
		Vec2 p2 = triangleVerts[(i + 1) % 3];
		Vec2 edge = p2 - p1;
		float t = std::max(0.0f, std::min(1.0f, Dot(circlePos - p1, edge) / Dot(edge, edge)));
		Vec2 closestPoint = p1 + t * edge;

		Vec2 d = circlePos - closestPoint;
		float distSquared = Dot(d, d);
		if (distSquared <= radius * radius)
		{
			float dist = sqrt(distSquared);
			Vec2 normal = dist > 0.0f ? d / dist : Vec2(1.0f, 0.0f);

			contacts[0].position = closestPoint;
			contacts[0].normal = normal;
			contacts[0].separation = dist - radius;
			return 1;
		}
	}

	return 0;
}

int CircleToCircle(Body* bodyA, Body* bodyB, Contact* contacts)
{
	Vec2 posA = bodyA->position;
	Vec2 posB = bodyB->position;
	float radiusA = bodyA->radius;
	float radiusB = bodyB->radius;

	Vec2 d = posB - posA;
	float distSquared = Dot(d, d);
	float radiusSum = radiusA + radiusB;


	if (distSquared <= radiusSum * radiusSum)
	{
		float dist = sqrt(distSquared);
		Vec2 normal = dist > 0.0f ? d / dist : Vec2(1.0f, 0.0f);

		normal *= radiusA;

		contacts[0].position = posA + normal;
		contacts[0].normal = normal;
		contacts[0].separation = dist - radiusSum;
		return 1;
	}
	return 0;
}

int Collide(Contact* contacts, Body* bodyA, Body* bodyB)
{
	if (bodyA->shape == BOX && bodyB->shape == BOX)
	{
		return BoxToBox(bodyA, bodyB, contacts);
	}
	else if (bodyA->shape == CIRCLE && bodyB->shape == CIRCLE)
	{
		return CircleToCircle(bodyA, bodyB, contacts);
	}
	else if ((bodyA->shape == BOX && bodyB->shape == CIRCLE) || (bodyA->shape == CIRCLE && bodyB->shape == BOX))
	{
		return BoxToCircle(bodyB, bodyA, contacts); 
	}
	else if (bodyA->shape == TRIANGLE && bodyB->shape == TRIANGLE)
	{
		return TriangleToTriangle(bodyA, bodyB, contacts);
	}
	else if ((bodyA->shape == CIRCLE && bodyB->shape == TRIANGLE) || (bodyA->shape == TRIANGLE && bodyB->shape == CIRCLE))
	{
		return CircleToTriangle(bodyA, bodyB, contacts);
		
	}
	else if ((bodyA->shape == BOX && bodyB->shape == TRIANGLE) || (bodyA->shape == TRIANGLE && bodyB->shape == BOX))
	{
		return BoxToTriangle(bodyA, bodyB, contacts);
	}
	else {
		return 0;
	}
}