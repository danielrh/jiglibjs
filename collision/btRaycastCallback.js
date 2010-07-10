/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#include <stdio.h>

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "btRaycastCallback.h"

btTriangleRaycastCallback::btTriangleRaycastCallback(const btVector3& from,const btVector3& to, unsigned int flags)
	:
	m_from(from),
	m_to(to),
   //@BP Mod
   m_flags(flags),
	m_hitFraction(btScalar(1.))
{

}
function btSetInterpolate3(v0,v1,rt,dest) {
    var s=1.0-rt;
    dest[0]=s*v0[0]+rt*v1[0];
    dest[1]=s*v0[1]+rt*v1[1];
    dest[2]=s*v0[2]+rt*v1[2];
}
function btTriangleRaycastCallback(m_from,m_to,m_flags,reportHit) {
    var m_hitFraction=1.0;
    
    if (m_flags===undefined) {
        m_flags=0;
    }
    function processTriangle(ax,ay,az,bx,by,bz,cx,cy,cz, partId, triangleIndex) {
        
        var v10=vec3.create([bx-ax,by-ay,bz-az]);
        var v20=vec3.create([cx-ax,cy-ay,cz-az]); 
    
        var triangleNormal=vec3.create(); 
        vec3.cross(v10,v20,triangleNormal);
	
        var dist = ax*triangleNormal[0]+ay*triangleNormal[1]+az*triangleNormal[2];//vec3.dot (a,triangleNormal)
        var dist_a = vec3.dot(triangleNormal,m_from);
        dist_a-= dist;
        var dist_b = vec3.dot(triangleNormal,m_to);
        dist_b -= dist;

        if ( dist_a * dist_b >= 0.0 )
        {
            return ; // same sign
        }
       //@BP Mod - Backface filtering
       if (((m_flags & kF_FilterBackfaces) != 0) && (dist_a > 0.0))
       {
          // Backface, skip check
          return;
       }
	
	   var proj_length=dist_a-dist_b;
	   var distance = (dist_a)/(proj_length);
	// Now we have the intersection point on the plane, we'll see if it's inside the triangle
	// Add an epsilon as a tolerance for the raycast,
	// in case the ray hits exacly on the edge of the triangle.
	// It must be scaled for the triangle size.
	
       if(distance < m_hitFraction)
       {
	   

           var edge_tolerance =vec3.dot(triangleNormal,triangleNormal);
           edge_tolerance *= -0.0001;
           btVector3 point=vec3.create(); 
           btSetInterpolate3( m_from, m_to, distance, point);
           {
               var v0p = vec3.create([ax-point[0],ay-point[1],az-point[2]]);
               var v1p = vec3.create([bx-point[0],by-point[1],bz-point[2]]);
               btVector3 cp0=vec3.create(); 
               vec3.cross(v0p,v1p,cp0);

               if ( vec3.dot(cp0,triangleNormal) >=edge_tolerance) 
			   {
						

                   var v2p=vec3.create([cx-point[0],cy-point[1],cz-point[2]]);
                   var cp1=vec3.create();
                   vec3.cross(v1p, v2p,cp1);
                   if ( vec3.dot(cp1,triangleNormal) >=edge_tolerance) 
				{
					var cp2=vec3.create();
					vec3.cross(v2p,v0p,cp2);
					
					if ( vec3.dot(cp2,triangleNormal) >=edge_tolerance) 
					{
                  //@BP Mod
                  // Triangle normal isn't normalized
                        vec3.normalize(triangleNormal);

                  //@BP Mod - Allow for unflipped normal when raycasting against backfaces
                  if (((m_flags & kF_KeepUnflippedNormal) != 0) || (dist_a <= 0.0))
						{
                            triangleNormal[0]=-triangleNormal[0];
                            triangleNormal[1]=-triangleNormal[1];
                            triangleNormal[2]=-triangleNormal[2];
							m_hitFraction = reportHit(triangleNormal,distance,partId,triangleIndex);
                            triangleNormal[0]=-triangleNormal[0];
                            triangleNormal[1]=-triangleNormal[1];
                            triangleNormal[2]=-triangleNormal[2];
						}
						else
						{
                     m_hitFraction = reportHit(triangleNormal,distance,partId,triangleIndex);
						}
					}
				}
			}
		}
	}
    }
    return processTriangle;
}


btTriangleConvexcastCallback::btTriangleConvexcastCallback (const btConvexShape* convexShape, const btTransform& convexShapeFrom, const btTransform& convexShapeTo, const btTransform& triangleToWorld, const btScalar triangleCollisionMargin)
{
	m_convexShape = convexShape;
	m_convexShapeFrom = convexShapeFrom;
	m_convexShapeTo = convexShapeTo;
	m_triangleToWorld = triangleToWorld;
	m_hitFraction = 1.0;
    m_triangleCollisionMargin = triangleCollisionMargin;
}
/**
 * @param {btConvexShape} m_convexShape
 * @param {mat4} m_convexShapeFrom
 * @param {mat4} m_convexShapeTo
 * @param {mat4} m_triangleToWorld
 * @param {number} m_triangleCollisionMargin
 */ 
function btTriangleConvexcastCallback(m_convexShape,m_convexShapeFrom,m_convexShapeTo,m_triangleToWorld,m_triangleCollisionMargin) {
  var m_hitFraction=1.0;
  function processTriangle (ax,ay,ax,bx,by,bz,cx,cy,cz,  partId, triangleIndex)
  {
    var triangleShape = new btTriangleShape ([ax,ay,ax],[bx,by,bz],[cx,cy,cz]);
    triangleShape.setMargin(m_triangleCollisionMargin);

	var simplexSolver = new btVoronoiSimplexSolver;
	var gjkEpaPenetrationSolver = new btGjkEpaPenetrationDepthSolver;

//#define  USE_SUBSIMPLEX_CONVEX_CAST 1
//if you reenable USE_SUBSIMPLEX_CONVEX_CAST see commented out code below
    var convexCaster;
    if (USE_SUBSIMPLEX_CONVEX_CAST) {
	  convexCaster=new btSubsimplexConvexCast(m_convexShape, triangleShape, simplexSolver);
    }else {
	//btGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
	  convexCaster=new btContinuousConvexCollision(m_convexShape,triangleShape, simplexSolver, gjkEpaPenetrationSolver);
    } //#USE_SUBSIMPLEX_CONVEX_CAST
	
	var castResult=btConvexCast.CastResult;
	castResult.m_fraction = 1.;
	if (convexCaster.calcTimeOfImpact(m_convexShapeFrom,m_convexShapeTo,m_triangleToWorld, m_triangleToWorld, castResult))
	{
		//add hit
		if (vec3.dot(castResult.m_normal,castResult.m_normal) > 0.0001)
		{					
			if (castResult.m_fraction < m_hitFraction)
			{
/* btContinuousConvexCast's normal is already in world space */
/*
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
				//rotate normal into worldspace
				castResult.m_normal = m_convexShapeFrom.getBasis() * castResult.m_normal;
#endif //USE_SUBSIMPLEX_CONVEX_CAST
*/
				castResult.m_normal.normalize();

				reportHit (castResult.m_normal,
							castResult.m_hitPoint,
							castResult.m_fraction,
							partId,
							triangleIndex);
			}
		}
	}
}
