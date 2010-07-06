/**
 * @param {!btStridingMeshInterface} meshInterface 
 */
function btTriangleMeshShape (meshInterface){
    this.m_meshInterface=meshInterface;
    this.m_localAabbMin=vec3.create([BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT]);
    this.m_localAabbMin=vec3.create([-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT]);
    if (meshInterface.hasPremadeAabb()){
        meshInterface.getPremadeAabb(m_localAabbMin,m_localAabbMax);
    }else {
        this.recalcLocalAabb();
    }
};

btTriangleMeshShape.prototype.getAabb=function(trans,aabbMin,aabbMax){
    var margin=this.getMargin();
	var localHalfExtents =     vec3.create([0.5*(this.m_localAabbMax[0]-this.m_localAabbMin[0])+margin,
                                           0.5*(this.m_localAabbMax[1]-this.m_localAabbMin[1])+margin,
                                           0.5*(this.m_localAabbMax[2]-this.m_localAabbMin[2])+margin]);

    var localCenter =vec3.create([0.5*(this.m_localAabbMax[0]+this.m_localAabbMin[0]),
                                 0.5*(this.m_localAabbMax[1]+this.m_localAabbMin[1]),
                                 0.5*(this.m_localAabbMax[2]+this.m_localAabbMin[2])]);
    
	
	var abs_b = mat3.create();
    mat4.toMat3(trans,abs_b);
    
	mat4.multiplyVec3(trans,
                      localCenter);
    
	mat3.multiplyVec3(abs_b,localHalfExtents);
	vec3.subtract(localCenter, localHalfExtent,aabbMin);
	vec3.add(localCenter,localHalfExtent,aabbMax);
};

btTriangleMeshShape.prototype.recalcLocalAabb=function()
{
    var vec=vec3.create();
	for (var i=0;i<3;i++)
	{
		vec[0]=vec[1]=vec[2]=0.0;
		vec[i] = 1.0;
		var tmp = this.localGetSupportingVertex(vec);
		this.m_localAabbMax[i] = tmp[i]+this.m_collisionMargin;
		vec[i] = -1.0;
		tmp = this.localGetSupportingVertex(vec);
		this.m_localAabbMin[i] = tmp[i]-this.m_collisionMargin;
	}
};

/**
 * @param {function} callback  function (ax,ay,az,bx,by,bz,cx,cy,cz,index) that processes triangle coordinates
 * @param {Array} aabbMin minimum vec3 for triangles under consideration
 * @param {Array} aabbMax max vec3 for triangles under consideration
 */
btTriangleMeshShape.prototype.processAllTriangles=function(callback,aabbMin,aabbMax) {
    function internalProcessTriangleIndex(ax,ay,az,bx,by,bz,cx,cy,cz,subPart,index){
        if (TestTriangleAgainstAabb2(ax,ay,az,bx,by,bz,cx,cy,cz,aabbMin,aabbMax)){
            callback(ax,ay,az,bx,by,bz,cx,cy,cz,subPart,index);
        }
    }
    this.m_meshInterface.InternalProcessAlltriangles(internalProcesstriangleIndex,aabbMin,aabbMax);
};


btTriangleMeshShape.prototype.calculateLocalInertia=function(mass,inertia) {
    inertia[0]=0;
    inertia[1]=0;
    inertia[2]=0;
};

btTriangleMeshShape.prototype.localGetSupportingVertex=function(vec) {
    var supportVertex=vec3.create();
    var maxDot=-BT_LARGE_FLOAT;
    function supportCallback(ax,ay,az,bx,by,bz,cx,cy,cz,subPart,index) 
{        var dot=vec[0]*ax+vec[1]*ay+vec[2]*az;
         if (dot>maxDot) {
             maxDot=dot;
             supportVertex[0]=ax;
             supportVertex[1]=ay;
             supportVertex[2]=az;
         }
         dot=vec[0]*bx+vec[1]*by+vec[2]*bz;
         if (dot>maxDot) {
             maxDot=dot;
             supportVertex[0]=bx;
             supportVertex[1]=by;
             supportVertex[2]=bz;
         }         
         dot=vec[0]*cx+vec[1]*cy+vec[2]*cz;
         if (dot>maxDot) {
             maxDot=dot;
             supportVertex[0]=cx;
             supportVertex[1]=cy;
             supportVertex[2]=cz;
         }
    }
    this.aabbMax=vec3.create([BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT]);//includes everything, so bypass processAllTriangles with bbox check
    this.aabbMin=vec3.create([-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT]);

    this.m_meshInterface.InternalProcessAlltriangles(supportCallback,aabbMin,aabbMax);
    return supportVertex;
};
btTriangleMeshShape.prototype.setLocalScaling=function(scaling) {
    this.m_meshInterface.setScaling(scaling);
    this.recalcLocalAabb();
};

btTriangleMeshShape.prototype.getLocalScaling=function() {
    return this.m_meshInterface.getScaling();
};
	
btTriangleMeshShape.prototype.getMeshInterface=function() 
{
	return this.m_meshInterface;
};
btTriangleMeshShape.prototype.getLocalAabbMin=function()
{
	return this.m_localAabbMin;
};
btTriangleMeshShape.getLocalAabbMax=function()
{
	return this.m_localAabbMax;
};
