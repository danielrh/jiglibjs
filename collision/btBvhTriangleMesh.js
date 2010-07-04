var BT_TRIANGLE_MESH_SHAPE_PROXYTYPE=1025;
function btBvhTriangleMeshShape (meshInterface, useQuantizedAabbCompression,bvhAabbMin,bvhAabbMax, buildBvh) {
    btTriangleMeshShape(meshInterface);
    this.m_bvh=null;
    /**
     * @type {Array}
     */
    this.m_triangleInfoMap=null;
    /**
     * @type {boolean}
     */
    this.m_useQuantizedAabbCompression=useQuantizedAabbCompression;
    /**
     * @type {number}
     */
    this.m_shapeType=BT_TRIANGLE_MESH_SHAPE_PROXYTYPE;
    /**
     * @type {btOptimizedBvh}
     */
    this.m_bvh=null;
    if (buildBvh) {
        this.m_bvh=btOptimizedBvh();
        this.m_bvh.build(meshInterface,useQuantizedAabbCompression,bvhAabbMin,bvhAabbMax);        
    }
}

btBvhTriangleMeshShape.prototype.partialRefitTree=function(aabbMin,aabbMax) {
    this.m_bvh.refitPartial(this.m_meshInterface,aabbMin,aabbMax);
    this.m_localAabbMin.setMin(aabbMin);
    this.m_localAabbMax.setMax(aabbMax);
};

btBvhTriangleMeshShape.prototype.refitTree=function(aabbMin,aabbMax) {
    this.m_bvh.refitPartial(this.m_meshInterface,aabbMin,aabbMax);
    this.recalcLocalAabb();
};

btBvhTriangleMeshShape.prototype.performRaycast=function(callback,raySource,rayTarget) {
    function nodeOverlapCallback(nodeTriangleIndex) {
        var meshInterface=this.m_meshInterface;
        var indexArray=meshInterface.m_indexArray;
        var vertexArray=meshInterface.m_vertexArray;
        var vertexStride=meshInterface.m_stride;
        var scale=meshInterface.m_scaling;
        var index0=indexArray[nodeTriangleIndex]*vertexStride;
        var index1=indexArray[nodeTriangleIndex+1]*vertexStride;
        var index2=indexArray[nodeTriangleIndex+2]*vertexStride;
        
        callback(vertexArray[index0]*scale[0],
                 vertexArray[index0+1]*scale[1],
                 vertexArray[index0+2]*scale[2],
                 vertexArray[index1]*scale[0],
                 vertexArray[index1+1]*scale[1],
                 vertexArray[index1+2]*scale[2],
                 vertexArray[index2]*scale[0],
                 vertexArray[index2+1]*scale[1],
                 vertexArray[index2+2]*scale[2],
                 nodeTriangleIndex);
    }
    this.m_bvh.reportRayOverlappingNodex(nodeOverlapCallback,
                                         raySource,
                                         rayTarget,
                                         aabbMin,
                                         aabbMax);
};


btBvhTriangleMeshShape.prototype.performConvexcast=function (callback, boxSource, boxTarget, aabbMin, aabbMax)
{
    function nodeOverlapCallback(nodeTriangleIndex) {
        var meshInterface=this.m_meshInterface;
        var indexArray=meshInterface.m_indexArray;
        var vertexArray=meshInterface.m_vertexArray;
        var vertexStride=meshInterface.m_stride;
        var scale=meshInterface.m_scaling;
        var index0=indexArray[nodeTriangleIndex]*vertexStride;
        var index1=indexArray[nodeTriangleIndex+1]*vertexStride;
        var index2=indexArray[nodeTriangleIndex+2]*vertexStride;
        
        callback(vertexArray[index0]*scale[0],
                 vertexArray[index0+1]*scale[1],
                 vertexArray[index0+2]*scale[2],
                 vertexArray[index1]*scale[0],
                 vertexArray[index1+1]*scale[1],
                 vertexArray[index1+2]*scale[2],
                 vertexArray[index2]*scale[0],
                 vertexArray[index2+1]*scale[1],
                 vertexArray[index2+2]*scale[2],
                 nodeTriangleIndex);
    }
    this.m_bvh.reportBoxCastOverlappingNodex(nodeOverlapCallback,
                                             boxSource,
                                             boxTarget,
                                             aabbMin,
                                             aabbMax);
};


btBvhTriangleMeshShape.prototype.processAllTriangles=function(callback,aabbMin,aabbMax) 
{
    function nodeOverlapCallback(nodeTriangleIndex) {
        var meshInterface=this.m_meshInterface;
        var indexArray=meshInterface.m_indexArray;
        var vertexArray=meshInterface.m_vertexArray;
        var vertexStride=meshInterface.m_stride;
        var scale=meshInterface.m_scaling;
        var index0=indexArray[nodeTriangleIndex]*vertexStride;
        var index1=indexArray[nodeTriangleIndex+1]*vertexStride;
        var index2=indexArray[nodeTriangleIndex+2]*vertexStride;
        
        callback(vertexArray[index0]*scale[0],
                 vertexArray[index0+1]*scale[1],
                 vertexArray[index0+2]*scale[2],
                 vertexArray[index1]*scale[0],
                 vertexArray[index1+1]*scale[1],
                 vertexArray[index1+2]*scale[2],
                 vertexArray[index2]*scale[0],
                 vertexArray[index2+1]*scale[1],
                 vertexArray[index2+2]*scale[2],
                 nodeTriangleIndex);
    }

	this.m_bvh.reportAabbOverlappingNodex (nodeOverlapCallback, aabbMin, aabbMax);
};



btBvhTriangleMeshShape.prototype.setLocalScaling=function(scaling)
{
   var scalingDiff=vec3.create();
   vec3.sub(this.m_meshInterface.getScaling(),scaling,scalignDiff);
   if (vec3.dot(scalingDiff,scalingDiff) > SIMD_EPSILON)
   {
      btTriangleMeshShape.prototype.setLocalScaling.call(this,scaling);
	  this.buildOptimizedBvh();
   }
};

btBvhTriangleMeshShape.prototype.buildOptimizedBvh=function()
{
	this.m_bvh = new btOptimizedBvh();
	//rebuild the bvh...
	m_bvh.build(this.m_meshInterface,this.m_useQuantizedAabbCompression,this.m_localAabbMin,this.m_localAabbMax);
};

btBvhTriangleMeshShape.prototype.setOptimizedBvh=function(bvh, scaling)
{

   this.m_bvh = bvh;
   // update the scaling without rebuilding the bvh
   var scalingDiff=vec3.create();
   vec3.sub(this.m_meshInterface.getScaling(),scaling,scalignDiff);
   if (vec3.dot(scalingDiff,scalingDiff) > SIMD_EPSILON)
   {
      btTriangleMeshShape.prototype.setLocalScaling.call(this,scaling);
   }
};

btBvhTriangleMeshShape.prototype.setTriangleInfoMap=function(triangleInfoMap) {
    this.m_triangleInfoMap=triangleInfoMap;
};

btBvhTriangleMeshShape.prototype.getTriangleInfoMap=function() {
    return this.m_triangleInfoMap;
};
