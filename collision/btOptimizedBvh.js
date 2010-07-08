/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

function btOptimizedBvh(){
    btQuantizedBvh.call(this);
}

for (var btQuantizedFunction in btQuantizedBvh.prototype){
    btOptimizedBvh.prototype[btQuantizedFunction]=btQuantizedBvh.prototype[btQuantizedFunction];
}




btOptimizedBvh.prototype.build=function(triangles, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax)
{
	var m_useQuantization = useQuantizedAabbCompression;

    var thus=this;
	// NodeArray	triangleNodes;
    function NodeTriangleCallback (ax,ay,az,bx,by,bz,cx,cy,cz, partId, triangleIndex)
		{
			var node=new btOptimizedBvhNode();
			var	aabbMin,aabbMax;
			aabbMin=vec3.create([BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT]);
			aabbMax=vec3.create([-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT]);
			if (aabbMin[0]>ax) aabbMin[0]=ax;
			if (aabbMin[1]>ay) aabbMin[1]=ay;
			if (aabbMin[2]>az) aabbMin[2]=az;

			if (aabbMax[0]<ax) aabbMax[0]=ax;
			if (aabbMax[1]<ay) aabbMax[1]=ay;
			if (aabbMax[2]<az) aabbMax[2]=az;

			if (aabbMin[0]>bx) aabbMin[0]=bx;
			if (aabbMin[1]>by) aabbMin[1]=by;
			if (aabbMin[2]>bz) aabbMin[2]=bz;

			if (aabbMax[0]<bx) aabbMax[0]=bx;
			if (aabbMax[1]<by) aabbMax[1]=by;
			if (aabbMax[2]<bz) aabbMax[2]=bz;

			if (aabbMin[0]>cx) aabbMin[0]=cx;
			if (aabbMin[1]>cy) aabbMin[1]=cy;
			if (aabbMin[2]>cz) aabbMin[2]=cz;

			if (aabbMax[0]<cx) aabbMax[0]=cx;
			if (aabbMax[1]<cy) aabbMax[1]=cy;
			if (aabbMax[2]<cz) aabbMax[2]=cz;


			//with quantization?
			node.m_aabbMinOrg = aabbMin;
			node.m_aabbMaxOrg = aabbMax;

			node.m_escapeIndex = -1;
	
			//for child nodes
			node.m_subPart = partId;
			node.m_triangleIndex = triangleIndex;
			thus.m_leafNodes.push(node);
		}
	function QuantizedNodeTriangleCallback(ax,ay,az,bx,by,bz,cx,cy,cz,partId,triangleIndex)
	{
			// The partId and triangle index must fit in the same (positive) integer
			if(partId >= (1<<btQuantizedBvhNode.MAX_NUM_PARTS_IN_BITS)&&console){
                console.log("Part Id "+partId+" out of bounds of quantized integers");
            }
			if(triangleIndex >= (1<<(31-btQuantizedBvhNode.MAX_NUM_PARTS_IN_BITS))&&console){
                console.log("Part Id "+triangleIndes+" out of bounds of quantized integers");
            }
			//negative indices are reserved for escapeIndex
			if (triangleIndex<0&&console){
                console.log("Triangle Index "+triangleIndex+" cannot be negative");
            }

			var node = new btQuantizedBvhNode();
			var aabbMin,aabbMax;
			aabbMin=vec3.create([BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT]);
			aabbMax=vec3.create([-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT]);
			if (aabbMin[0]>ax) aabbMin[0]=ax;
			if (aabbMin[1]>ay) aabbMin[1]=ay;
			if (aabbMin[2]>az) aabbMin[2]=az;

			if (aabbMax[0]<ax) aabbMax[0]=ax;
			if (aabbMax[1]<ay) aabbMax[1]=ay;
			if (aabbMax[2]<az) aabbMax[2]=az;

			if (aabbMin[0]>bx) aabbMin[0]=bx;
			if (aabbMin[1]>by) aabbMin[1]=by;
			if (aabbMin[2]>bz) aabbMin[2]=bz;

			if (aabbMax[0]<bx) aabbMax[0]=bx;
			if (aabbMax[1]<by) aabbMax[1]=by;
			if (aabbMax[2]<bz) aabbMax[2]=bz;

			if (aabbMin[0]>cx) aabbMin[0]=cx;
			if (aabbMin[1]>cy) aabbMin[1]=cy;
			if (aabbMin[2]>cz) aabbMin[2]=cz;

			if (aabbMax[0]<cx) aabbMax[0]=cx;
			if (aabbMax[1]<cy) aabbMax[1]=cy;
			if (aabbMax[2]<cz) aabbMax[2]=cz;

			//PCK: add these checks for zero dimensions of aabb
			var MIN_AABB_DIMENSION = 0.002;
			var MIN_AABB_HALF_DIMENSION = 0.001;
			if (aabbMax[0] - aabbMin[0] < MIN_AABB_DIMENSION)
			{
				aabbMax[0]=aabbMax[0] + MIN_AABB_HALF_DIMENSION;
				aabbMin[0]=aabbMin[0] - MIN_AABB_HALF_DIMENSION;
			}
			if (aabbMax[1] - aabbMin[1] < MIN_AABB_DIMENSION)
			{
				aabbMax[1]=aabbMax[1] + MIN_AABB_HALF_DIMENSION;
				aabbMin[1]=aabbMin[1] - MIN_AABB_HALF_DIMENSION;
			}
			if (aabbMax[2] - aabbMin[2] < MIN_AABB_DIMENSION)
			{
				aabbMax[2]=aabbMax[2] + MIN_AABB_HALF_DIMENSION;
				aabbMin[2]=aabbMin[2] - MIN_AABB_HALF_DIMENSION;
			}

			thus.quantize(node.m_quantizedAabbMin,aabbMin,0);
			thus.quantize(node.m_quantizedAabbMax,aabbMax,1);

			node.m_escapeIndexOrTriangleIndex = (partId<<(31-btQuantizedBvhNode.MAX_NUM_PARTS_IN_BITS)) | triangleIndex;

			thus.m_quantizedLeafNodes.push(node);
		
	}
	


	var numLeafNodes = 0;

	
	if (this.m_useQuantization)
	{

		//initialize quantization values
		this.setQuantizationValues(bvhAabbMin,bvhAabbMax);

	
		triangles.InternalProcessAllTriangles(QuantizedNodeTriangleCallback,this.m_bvhAabbMin,this.m_bvhAabbMax);

		//now we have an array of leafnodes in m_leafNodes
		numLeafNodes = this.m_quantizedLeafNodes.length;


		this.m_quantizedContiguousNodes.length=2*numLeafNodes;
        for (var i=0;i<2*numLeafNodes;++i){
            this.m_quantizedContiguousNodes[i]=new btQuantizedBvhNode();    
        }
	} else
	{

		var aabbMin=vec3.create([-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT]);
		var aabbMax=vec3.create([BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT]);

		triangles.InternalProcessAllTriangles(NodeTriangleCallback,aabbMin,aabbMax);

		//now we have an array of leafnodes in m_leafNodes
		numLeafNodes = this.m_leafNodes.size();

		this.m_contiguousNodes.resize(2*numLeafNodes);
	}

	this.m_curNodeIndex = 0;

	this.buildTree(0,numLeafNodes);

	///if the entire tree is small then subtree size, we need to create a header info for the tree
	if(this.m_useQuantization && !this.m_SubtreeHeaders.length)
	{
        var subtree;
		this.m_SubtreeHeaders.push(subtree = new btBvhSubtreeInfo());
		subtree.setAabbFromQuantizeNode(this.m_quantizedContiguousNodes[0]);
		subtree.m_rootNodeIndex = 0;
		subtree.m_subtreeSize = this.m_quantizedContiguousNodes[0].isLeafNode() ? 1 : this.m_quantizedContiguousNodes[0].getEscapeIndex();
	}

	//PCK: update the copy of the size
	this.m_subtreeHeaderCount = this.m_SubtreeHeaders.length;

	//PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
	this.m_quantizedLeafNodes=[];
	this.m_leafNodes=[];
};




btOptimizedBvh.prototype.refit=function(meshInterface, aabbMin, aabbMax)
{
	if (this.m_useQuantization)
	{

		this.setQuantizationValues(aabbMin,aabbMax);

		this.updateBvhNodes(meshInterface,0,m_curNodeIndex,0);

		///now update all subtree headers

		var i;
        var subtreeLen=this.m_SubtreeHeaders.length;
		for (i=0;i<subtreeLen;i++)
		{
			var subtree = this.m_SubtreeHeaders[i];
			subtree.setAabbFromQuantizeNode(this.m_quantizedContiguousNodes[subtree.m_rootNodeIndex]);
		}

	} else
	{

	}
};




btOptimizedBvh.prototype.refitPartial=function(meshInterface, aabbMin, aabbMax)
{
	//incrementally initialize quantization values
	if ((!m_useQuantization)&&console) {
        console.log("Partial refit called without being a quantized tree");
    }    
	if(aabbMin[0] <= m_bvhAabbMin[0]&&console){
        console.log("aabbMin outside internal tree's aabbMin");            
    }
	if(aabbMin[1] <= m_bvhAabbMin[1]&&console){
        console.log("aabbMin outside internal tree's aabbMin");            
    }
	if(aabbMin[2] <= m_bvhAabbMin[2]&&console){
        console.log("aabbMin outside internal tree's aabbMin");            
    }
    
	if(aabbMax[0] >= m_bvhAabbMax[0]&&console){
        console.log("aabbMax outside internal tree's aabbMax");            
    }
	if(aabbMax[1] >= m_bvhAabbMax[1]&&console){
        console.log("aabbMax outside internal tree's aabbMax");            
    }
	if(aabbMax[2] >= m_bvhAabbMax[2]&&console){
        console.log("aabbMax outside internal tree's aabbMax");            
    }
    
	///we should update all quantization values, using updateBvhNodes(meshInterface);
	///but we only update chunks that overlap the given aabb
	
	var quantizedQueryAabbMin=vec3.create();
	var quantizedQueryAabbMax=vec3.create();

	this.quantize(quantizedQueryAabbMin,aabbMin,0);
	this.quantize(quantizedQueryAabbMax,aabbMax,1);

	var i;
    var subtreeLen=this.m_SubtreeHeaders.size();
	for (i=0;i<subtreeLen;i++)
	{
		btBvhSubtreeInfo& subtree = m_SubtreeHeaders[i];

		//PCK: unsigned instead of bool
		var overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
		if (overlap != 0)
		{
			this.updateBvhNodes(meshInterface,subtree.m_rootNodeIndex,subtree.m_rootNodeIndex+subtree.m_subtreeSize,i);

			subtree.setAabbFromQuantizeNode(this.m_quantizedContiguousNodes[subtree.m_rootNodeIndex]);
		}
	}
	
};

btOptimizedBvh.prototype.updateBvhNodes=function(meshInterface, firstNode, endNode, index)
{

	if ((!m_useQuantization)&&console) {
      console.log("Updating BvhNodes when not quantized");
    }

	var curNodeSubPart=-1;

	//get access info to trianglemesh data
	var numverts = 0;
	var stride = 0;
	var indexstride = 0;
	var numfaces = 0;
    
	var triangleVerts=[vec3.create(),vec3.create(),vec3.create()];
	var aabbMin=vec3.create(),aabbMax=vec3.create();
	var meshScaling = meshInterface.getScaling();
    var indexStride=meshInterface.m_indexStride;
	var indexArray;
    var vertexStride=meshInterface.m_vertexStride;
    var vertexArray;
	var i;
	for (i=endNode-1;i>=firstNode;i--)
	{


			var curNode = this.m_quantizedContiguousNodes[i];
			if (curNode.isLeafNode())
			{
				//recalc aabb from triangle data
				var nodeSubPart = curNode.getPartId();
				var nodeTriangleIndex = curNode.getTriangleIndex();
				if (nodeSubPart != curNodeSubPart)
				{
					if (curNodeSubPart >= 0)
						meshInterface.unLockReadOnlyVertexBase(curNodeSubPart);
                    vertexArray=meshInterface.getVertexArray(nodeSubPart);
                    indexArray=meshInterface.getIndexArray(nodeSubPart);
                    
					curNodeSubPart = nodeSubPart;
				}
				//triangles->getLockedReadOnlyVertexIndexBase(vertexBase,numVerts,
                var j;
                for (j=0;j<3;++j) {				
                    aabbMin[j]=BT_LARGE_FLOAT;
                    aabbMax[j]=-BT_LARGE_FLOAT;
                }
				for (j=2;j>=0;j--)
				{
					
                    var vertexId=vertexStride*indexArray[nodeTriangleIndex];
					triangleVerts[j][0] = vertexArray[vertexId]*meshScaling[0];
					triangleVerts[j][1] = vertexArray[vertexId+1]*meshScaling[1];
					triangleVerts[j][2] = vertexArray[vertexId+2]*meshScaling[2];
                    for (var k=0;k<3;++k) {
                        if (aabbMin[k]>triangleVerts[j][k])
                            aabbMin[k]=triangleVerts[j][k];
                        if (aabbMax[k]<triangleVerts[j][k])
                            aabbMax[k]=triangleVerts[j][k];
                    }
                    
				}

				this.quantize(curNode.m_quantizedAabbMin[0],aabbMin,0);
				this.quantize(curNode.m_quantizedAabbMax[0],aabbMax,1);
				
			} else
			{
				//combine aabb from both children

				var leftChildNode = this.m_quantizedContiguousNodes[i+1];
				
				var rightChildNode = leftChildNode.isLeafNode() ? this.m_quantizedContiguousNodes[i+2] :
					this.m_quantizedContiguousNodes[i+1+leftChildNode.getEscapeIndex()];
				

				{
					for (var ind=0;ind<3;ind++)
					{
						curNode.m_quantizedAabbMin[ind] = leftChildNode.m_quantizedAabbMin[ind];
						if (curNode.m_quantizedAabbMin[ind]>rightChildNode.m_quantizedAabbMin[ind])
							curNode.m_quantizedAabbMin[ind]=rightChildNode.m_quantizedAabbMin[ind];

						curNode.m_quantizedAabbMax[ind] = leftChildNode.m_quantizedAabbMax[ind];
						if (curNode.m_quantizedAabbMax[ind] < rightChildNode.m_quantizedAabbMax[ind])
							curNode.m_quantizedAabbMax[ind] = rightChildNode.m_quantizedAabbMax[ind];
					}
				}
			}

		}

		if (curNodeSubPart >= 0)
			meshInterface.unLockReadOnlyVertexBase(curNodeSubPart);

		
};

