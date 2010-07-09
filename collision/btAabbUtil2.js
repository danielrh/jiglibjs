/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


function AabbExpand (aabbMin,
					aabbMax,
					expansionMin,
					expansionMax)
{
	vec3.add(aabbMin, expansionMin);
	vec3.add(aabbMax, expansionMax);
}

/// conservative test for overlap between two aabbs
function TestPointAgainstAabb2(aabbMin1, aabbMax1,
								point)
{
	var overlap = true;
	overlap = (aabbMin1[0] > point[0] || aabbMax1[0] < point[0]) ? false : overlap;
	overlap = (aabbMin1[2] > point[2] || aabbMax1[2] < point[2]) ? false : overlap;
	overlap = (aabbMin1[1] > point[1] || aabbMax1[1] < point[1]) ? false : overlap;
	return overlap;
}


/// conservative test for overlap between two aabbs
function TestAabbAgainstAabb2(aabbMin1, aabbMax1,
							aabbMin2, aabbMax2)
{
	var overlap = true;
	overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? false : overlap;
	overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? false : overlap;
	overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? false : overlap;
	return overlap;
}
function btMin(a,b) {
    return (a<b?a:b);
}
function btMax(a,b) {
    return (a>b?a:b);
}

/// conservative test for overlap between triangle and aabb
function TestTriangleAgainstAabb2(ax,ay,az,bx,by,bz,cx,cy,cz,
									aabbMin, aabbMax)
{
	if (btMin(btMin(ax, bx), cx) > aabbMax[0]) return false;
	if (btMax(btMax(ax, bx), cx) < aabbMin[0]) return false;

	if (btMin(btMin(az, bz), cz) > aabbMax[2]) return false;
	if (btMax(btMax(az, bz), cz) < aabbMin[2]) return false;
  
	if (btMin(btMin(ay, by), cy) > aabbMax[1]) return false;
	if (btMax(btMax(ay, by), cy) < aabbMin[1]) return false;
	return true;
}


function btOutcode(p, halfExtent) 
{
	return (p[0]  < -halfExtent[0] ? 0x01 : 0x0) |    
		   (p[0] >  halfExtent[0] ? 0x08 : 0x0) |
		   (p[1] < -halfExtent[1] ? 0x02 : 0x0) |    
		   (p[1] >  halfExtent[1] ? 0x10 : 0x0) |
		   (p[2] < -halfExtent[2] ? 0x4 : 0x0) |    
		   (p[2] >  halfExtent[2] ? 0x20 : 0x0);
}


/**
 * @param {Float32Array} rayFrom a vector of length 3 specifying the ray's starting loc
 * @param {Float32Array} rayInvDirection the inverse of the ray's direction
 * @param {Int32Array} raySign the sign of each direction of the ray (+1 for pos, 0 for neg)
 * @param {Array} bounds an array of Float32Array (length 3) vec3's indicating the bounds
 * @return {Array} tmin 1 element array to facilitate returning tmin
 * @param {number} lambda_min
 * @param {number} lambda_max
 */
function btRayAabb2(rayFrom,
								  rayInvDirection,
								  raySign,
								  bounds,
								  tmin,
								  lambda_min,
								  lambda_max)
{
	var tmax, tymin, tymax, tzmin, tzmax;
	tmin[0] = (bounds[raySign[0]][0] - rayFrom[0]) * rayInvDirection[0];
	tmax = (bounds[1-raySign[0]][0] - rayFrom[0]) * rayInvDirection[0];
	tymin = (bounds[raySign[1]][1] - rayFrom[1]) * rayInvDirection[1];
	tymax = (bounds[1-raySign[1]][1] - rayFrom[1]) * rayInvDirection[1];

	if ( (tmin[0] > tymax) || (tymin > tmax) )
		return false;

	if (tymin > tmin[0])
		tmin[0] = tymin;

	if (tymax < tmax)
		tmax = tymax;

	tzmin = (bounds[raySign[2]][2] - rayFrom[2]) * rayInvDirection[2];
	tzmax = (bounds[1-raySign[2]][2] - rayFrom[2]) * rayInvDirection[2];

	if ( (tmin[0] > tzmax) || (tzmin > tmax) )
		return false;
	if (tzmin > tmin[0])
		tmin[0] = tzmin;
	if (tzmax < tmax)
		tmax[0] = tzmax;
	return ( (tmin[0] < lambda_max) && (tmax > lambda_min) );
}
/**
 * @param {Float32Array} rayFrom start of ray
 * @param {Float32Array} rayFrom end of ray
 * @param {Float32Array} aabbMin smallest of the bbox
 * @param {Float32Array} aabbMax biggest of the bbox
 * @param {Array} param Array of size 1 holding location along the ray
 * @param {Float32Array} normal the normal of the triangle being intersected
 */
function btRayAabb(rayFrom, 
								 rayTo, 
								 aabbMin, 
								 aabbMax,
					  param, normal) 
{
	var aabbHalfExtent = vec3.create();
    vec3.subtract(aabbMax,aabbMin,aabbHalfExtent);
    vec3.scale(aabbHalfExtent,0.5);
    var aabbCenter=vec3.create();
	vec3.add(aabbMax,aabbMin,aabbCenter);
    vec3.scale(aabbCenter,0.5);
    var source = vec3.create();
    var target = vec3.create();
    vec3.subtract(rayFrom,aabbCenter,source);
    vec3.subtract(rayTo,aabbCenter,target);
	var	sourceOutcode = btOutcode(source,aabbHalfExtent);
	var targetOutcode = btOutcode(target,aabbHalfExtent);
	if ((sourceOutcode & targetOutcode) == 0x0)
	{
		var lambda_enter = 0.0;
		var lambda_exit  = param[0];
		var r = vec3.create();
        vec3.subtract(target, source,r);
		var i;
		var	normSign = 1;
		var hitNormal=vec3.create([0,0,0]);
		var bit=1;

		for (var j=0;j<2;j++)
		{
			for (i = 0; i != 3; ++i)
			{
				if (sourceOutcode & bit)
				{
					var lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
					if (lambda_enter <= lambda)
					{
						lambda_enter = lambda;
						hitNormal[0]=0;
                        hitNormal[1]=0;
                        hitNormal[2]=0;
						hitNormal[i] = normSign;
					}
				}
				else if (targetOutcode & bit) 
				{
					var lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
					lambda_exit=btMin(lambda_exit, lambda);
				}
				bit<<=1;
			}
			normSign = -1.0;
		}
		if (lambda_enter <= lambda_exit)
		{
			param[0] = lambda_enter;
			normal = hitNormal;
			return true;
		}
	}
	return false;
}


/**
 * @param {Float32Array} halfExtents
 * @param {number} margin
 * @param {Float32Array} t Transformation (tyep mat4)
 * @param {Float32Array} aabbMinOut
 * @param {Float32Array} aabbMaxOut
 */
function btTransformAabb(halfExtents, margin, t, aabbMinOut, aabbMaxOut)
{
	var halfExtentsWithMargin = vec3.create();
    halfExtentsWithMargin[0]=halfExtents[0]+margin;
    halfExtentsWithMargin[1]=halfExtents[1]+margin;
    halfExtentsWithMargin[2]=halfExtents[2]+margin;
	var abs_b = mat3.create();
    mat4.toMat3(t,abs_b);
	var center = vec3.create();
    mat4.multiplyVec3(t,center);
	var extent = vec3.create();
    mat3.multiplyVec3(abs_b,halfExtentsWithMargin,extent);
	vec3.subtract(center,extent,aabbMinOut);
	vec3.add(center,extent,aabbMaxOut);
}

/* DRH - Redundant?
SIMD_FORCE_INLINE	void btTransformAabb(const btVector3& localAabbMin,const btVector3& localAabbMax, btScalar margin,const btTransform& trans,btVector3& aabbMinOut,btVector3& aabbMaxOut)
{
		btAssert(localAabbMin[0] <= localAabbMax[0]);
		btAssert(localAabbMin[1] <= localAabbMax[1]);
		btAssert(localAabbMin[2] <= localAabbMax[2]);
		btVector3 localHalfExtents = btScalar(0.5)*(localAabbMax-localAabbMin);
		localHalfExtents+=btVector3(margin,margin,margin);

		btVector3 localCenter = btScalar(0.5)*(localAabbMax+localAabbMin);
		btMatrix3x3 abs_b = trans.getBasis().absolute();  
		btVector3 center = trans(localCenter);
		btVector3 extent = btVector3(abs_b[0].dot(localHalfExtents),
			   abs_b[1].dot(localHalfExtents),
			  abs_b[2].dot(localHalfExtents));
		aabbMinOut = center-extent;
		aabbMaxOut = center+extent;
}
*/
function testQuantizedAabbAgainstQuantizedAabb(aabbMin1,aabbMax1,aabbMin2,aabbMax2)
	{
		var overlap = true;
		overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? false : overlap;
		overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? false : overlap;
		overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? false : overlap;
		return overlap;
	}



