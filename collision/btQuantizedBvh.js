function btQuantizedBvhNode() {
    this.m_quantizedAabbMin=vec3.create();
    this.m_quantizedAabbMax=vec3.create();
    this.m_escapeIndexOrTriangleIndex=0;    
}

btQuantizedBvhNode.prototype.isLeafNode=function() {
    return this.m_escapeIndexOrTriangleIndex>=0;
};
btQuantizedBvhNode.prototype.getEscapeIndex=function() {
    return -this.m_escapeIndexOrTriangleIndex;
};
btQuantizedBvhNode.prototype.getTriangleIndex=function() {
    return (this.m_escapeIndexOrTriangleIndex&((1<<21)-1));
};
btQuantizedBvhNode.prototype.getPartId=function() {
    return (this.m_escapeIndexOrTriangleIndex>>21);
};

function btOptimizedBvhNode () {
    this.m_aabbMinOrg=vec3.create();
    this.m_aabbMaxOrg=vec3.create();
    this.m_escapeIndex=0;
    this.m_subPart=0;
    this.m_triangleIndex=0;
}



///btBvhSubtreeInfo provides info to gather a subtree of limited size
function btBvhSubtreeInfo () {
	this.m_quantizedAabbMin=vec3.create();
	this.m_quantizedAabbMax=vec3.create();
	this.m_rootNodeIndex=0;
	this.m_subtreeSize=0;
}
btBvhSubtreeInfo.setAabbFromQuantizeNode=function(quantizedNode)
	{
		vec3.set(quantizedNode.m_quantizedAabbMin,m_quantizedAabbMin);
		vec3.set(quantizedNode.m_quantizedAabbMax,m_quantizedAabbMax);
	};


function btQuantizedBvh() {
    this.m_bvhAabbMin=vec3.create([-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY]);
    this.m_bvhAabbMax=vec3.create([SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY]);
    this.m_bvhQuantization=vec3.create();
    this.m_bulletVersion=0.01;
    this.m_curNodeIndex=0;
    this.m_useQuantization=true;
    this.m_leafNodes=[];
    this.m_contiguousNodes=[];
    this.m_quantizedLeafNodes=[];
    this.m_quantizedContiguousNodes=[];
    this.m_traversalMode=btQuantizedBvh.TRAVERSAL_STACKLESS;    
    this.m_SubtreeHeaders=[];

    this.m_subtreeHeaderCount=0;
}

btQuantizedBvh.TRAVERSAL_STACKLESS=0;
btQuantizedBvh.TRAVERSAL_STACKLESS_CACHE_FRIENDLY=1;
btQuantizedBvh.TRAVERSAL_RECURSIVE=2;

	///two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
	///this might be refactored into a virtual, it is usually not calculated at run-time
btQuantizedBvh.prototype.setInternalNodeAabbMin=function(nodeIndex, aabbMin)
	{
		if (this.m_useQuantization)
		{
			this.quantize(this.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin ,aabbMin,0);
		} else
		{
			this.m_contiguousNodes[nodeIndex].m_aabbMinOrg = aabbMin;

		}
	};
btQuantizedBvh.prototype.setInternalNodeAabbMax=function(nodeIndex,aabbMax)
	{
		if (this.m_useQuantization)
		{
			this.quantize(this.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0],aabbMax,1);
		} else
		{
			this.m_contiguousNodes[nodeIndex].m_aabbMaxOrg = aabbMax;
		}
	}

btQuantizedBvh.prototype.getAabbMin=function(nodeIndex)
	{
		if (this.m_useQuantization)
		{
			return this.unQuantize(m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMin);
		}
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMinOrg;

	};
btQuantizedBvh.prototype.getAabbMax=function(nodeIndex)
	{
		if (this.m_useQuantization)
		{
			return this.unQuantize(m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMax);
		} 
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMaxOrg;
		
	};

	
	btQuantizedBvh.prototype.setInternalNodeEscapeIndex=function(nodeIndex, escapeIndex)
	{
		if (m_useQuantization)
		{
			m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = -escapeIndex;
		} 
		else
		{
			m_contiguousNodes[nodeIndex].m_escapeIndex = escapeIndex;
		}

	};

	btQuantizedBvh.prototype.mergeInternalNodeAabb=function(nodeIndex,newAabbMin,newAabbMax) 
	{
		if (this.m_useQuantization)
		{
			var quantizedAabbMin=vec3.create();
			var quantizedAabbMax=vec3.create();
			quantize(quantizedAabbMin,newAabbMin,0);
			quantize(quantizedAabbMax,newAabbMax,1);
			for (var i=0;i<3;i++)
			{
				if (this.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] > quantizedAabbMin[i])
					this.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] = quantizedAabbMin[i];

				if (this.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] < quantizedAabbMax[i])
					this.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] = quantizedAabbMax[i];

			}
		} else
		{
			//non-quantized
			for (var i=0;i<3;i++)
			{
				if (this.m_contiguousNodes[nodeIndex].m_aabbMin[i] > newAabbMin[i])
					this.m_contiguousNodes[nodeIndex].m_aabbMin[i] = newAabbMin[i];

				if (this.m_contiguousNodes[nodeIndex].m_aabbMax[i] < newAabbMax[i])
					this.m_contiguousNodes[nodeIndex].m_aabbMax[i] = newAabbMax[i];
			}
		}
	};

btQuantizedBvh.prototype.quantize=function(out,point,isMax) {
    vec3.set(point,out);
};
btQuantizedBvh.prototype.quantizeWithClamp=btQuantizedBvh.prototype.quantize;

btQuantizedBvh.prototype.unQuantize=function(vecIn)
	{
        return vecIn;
	};


	///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
	btQuantizedBvh.prototype.setTraversalMode=function(btTraversalMode	traversalMode)
	{
		this.m_traversalMode = traversalMode;
	}


	btQuantizedBvh.prototype.getQuantizedNodeArray=function()
	{	
		return	this.m_quantizedContiguousNodes;
	};


	btQuantizedBvh.prototype.getSubtreeInfoArray=function()
	{
		return this.m_SubtreeHeaders;
	};

	btQuantizedBvh.prototype.isQuantized=function()
	{
		return this.m_useQuantization;
	};

