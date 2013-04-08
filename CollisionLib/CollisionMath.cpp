#include "CollisionMath.h"
#include "CollisionShape.h"
#include <vector>
#include <cassert>

namespace
{
	// A maximum iteration count for GJK to avoid infinite loops
	// in case the algorithm fails to converge.
	const unsigned int GJK_MAX_ITER = 25;
}

/* Calculates the supporting point (farthest point in the given direction)
 * in the convex hull of the "Minkowski difference" of shape1 and shape2
 * (the Minkowski sum shape1 + (-shape2)).
 */
Point3D Support(const CollisionShape& shape1, const CollisionShape& shape2, const Vector3D& direction)
{
	Point3D p1 = shape1.GetFarthestPointInDirection(direction);
	Point3D p2 = shape2.GetFarthestPointInDirection(direction);
	return p1 - p2;
}

void UpdateForLine(std::vector<Point3D>& simplex, Vector3D& direction)
{
	/* In all UpdateFor<Simplex> functions, the variable A represents
	 * the point that was added to the simplex last and O represents
	 * the origin.
	 */
	assert(simplex.size() == 2);
	Point3D A = simplex[1];
	Point3D B = simplex[0];
	Vector3D AB = B = A;
	Vector3D AO = -A;

	// TODO: what happens here?
	if (AB.dot(AO) > 0)
	{
		direction = AB.cross(AO).cross(AB);
	}
	else
	{
		direction = AO;
	}
}

void UpdateForTriangle(std::vector<Point3D>& simplex, Vector3D& direction)
{
	assert(simplex.size() == 3);
	Point3D A = simplex[2];
	Point3D B = simplex[1];
	Point3D C = simplex[0];

	Vector3D AB = B - A;
	Vector3D AC = C - A;
	Vector3D AO = -A;

	Plane3D ABC = AB.cross(AC).normalized();

	// TODO: rename
	auto s = ABC.cross(AC).dot(AO);

	if (s > 0 && AC.dot(AO) > 0)
	{
		simplex.erase(simplex.begin() + 1); // remove the vertex B
		direction = AC.cross(AO).cross(AC);
		return;
	}

	if (s > 0 || AB.cross(ABC).dot(AO) > 0)
	{
		if (AB.dot(AO) > 0)
		{
			simplex.erase(simplex.begin()); // remove the vertex C
			direction = AB.cross(AO).cross(AB);
		}
		else
		{
			simplex.erase(simplex.begin(), simplex.begin() + 1); // remove all except A
			direction = AO;
		}
	}
	else if (ABC.dot(AO) > 0)
	{
		direction = ABC;
	}
	else
	{ // TODO: what effect does this reordering of vertices have?
		simplex.clear();
		simplex.push_back(B);
		simplex.push_back(C);
		simplex.push_back(A);
		direction = -ABC;
	}
}

// Treats 0 as a positive number.
int signum(double value)
{
	return (value < 0) ? -1 : 1;
}

/* Returns true if the origin is inside the tetrahedron
 * (there is a collision) and false otherwise.
 */
bool UpdateForTetrahedron(std::vector<Point3D>& simplex, Vector3D& direction)
{
	assert(simplex.size() == 4);
	Point3D A = simplex[3];
	Point3D B = simplex[2];
	Point3D C = simplex[1];
	Point3D D = simplex[0];

	Vector3D AO = -A;
	Vector3D AB = B - A;
	Vector3D AC = C - A;
	Vector3D AD = D - A;

	// The sides of the simplex that contain A (the last
	// support vertex added).
	Plane3D ABC = AB.cross(AC);
	Plane3D ACD = AC.cross(AD);
	Plane3D ADB = AD.cross(AB);

	// The signs of each of the points B, C and D relative
	// to the side opposite to them (ACD, ADB and ABC respectively).
	int signB = signum(ACD.dot(AB));
	int signC = signum(ADB.dot(AC));
	int signD = signum(ABC.dot(AD));

	// The sign of the origin relative to each of the sides
	// of the simplex that contain A.
	int signOriginABC = signum(ABC.dot(AO));
	int signOriginACD = signum(ACD.dot(AO));
	int signOriginADB = signum(ADB.dot(AO));

	bool originInsideSimplex =
		signB == signOriginACD &&
		signC == signOriginADB &&
		signD == signOriginABC;

	if (originInsideSimplex) // collision found
	{
		return true;
	}

	/* Look for the simplex vertex that is farthest from the
	 * origin and remove it from the simplex (cannot be A
	 * since it was the last vertex that was added).
	 * The new search direction is the plane normal of of
	 * the side opposite to the removed vertex.
	 *
	 * After this the simplex is a triangle again.
	 */
	if (signB != signOriginACD)
	{
		simplex.erase(simplex.begin() + 2);
		direction = ACD * signB;
	}
	else if (signC != signOriginADB)
	{
		simplex.erase(simplex.begin() + 1);
		direction = ADB * signC;
	}
	else
	{
		simplex.erase(simplex.begin());
		direction = ABC * signD;
	}
	UpdateForTriangle(simplex, direction);
	return false;
}

/* Checks whether the current simplex contains the origin.
 * If it does, there was an intersection and the function returns true.
 * Otherwise updates the simplex, selects a new search direction and
 * returns false (search must continue).
 */
bool CheckForCollisionAndUpdateSearch(std::vector<Point3D>& simplex, Vector3D& direction)
{
	assert(simplex.size() > 1 && simplex.size() < 5);
	switch (simplex.size())
	{
	case 2:
		UpdateForLine(simplex, direction);
	case 3:
		UpdateForTriangle(simplex, direction);
	case 4: // this is the only case where we test for intersections
		return UpdateForTetrahedron(simplex, direction);
	}
	return false;
}

bool GJKCollide(const CollisionShape& shape1, const CollisionShape& shape2)
{
	Vector3D d = Vector3D(1,0,0); // select the starting direction of the search
	std::vector<Point3D> simplex;
	simplex.push_back(Support(shape1, shape2, d));
	d = -d;
	for (int i = 0; i < GJK_MAX_ITER; i++)
	{
		Point3D p = Support(shape1, shape2, d);

		/* If the latest support vertex is not past the origin in the current
		 * search direction, the Minkowski sum cannot contain the origin
		 * and therefore the two shapes do not intersect.
		 *
		 * This is checked by taking the dot product of the support vertex
		 * with the current search direction (= the projection of the vertex
		 * onto the search direction vector) and comparing it with the
		 * projection of the origin (= 0).
		 */
		if (p.dot(d) <= 0)
		{
			return false;
		}

		simplex.push_back(p);

		if (CheckForCollisionAndUpdateSearch(simplex, d))
		{
			return true;
		}
	}
	return false;
}