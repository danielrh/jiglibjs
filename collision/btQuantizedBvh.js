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
btQuantizedBvhNode.MAX_NUM_PARTS_IN_BITS=(31-21);

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
btBvhSubtreeInfo.prototype.setAabbFromQuantizeNode=function(quantizedNode)
	{
		vec3.set(quantizedNode.m_quantizedAabbMin,this.m_quantizedAabbMin);
		vec3.set(quantizedNode.m_quantizedAabbMax,this.m_quantizedAabbMax);
	};


function btQuantizedBvh() {
    var SIMD_INFINITY=2.0*(1<<31)*2.0*(1<<31)*(1<<31);
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
			this.quantize(this.m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax,aabbMax,1);
		} else
		{
			this.m_contiguousNodes[nodeIndex].m_aabbMaxOrg = aabbMax;
		}
	};

btQuantizedBvh.prototype.getAabbMin=function(nodeIndex)
	{
		if (this.m_useQuantization)
		{
			return this.unQuantize(this.m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMin);
		}
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMinOrg;

	};
btQuantizedBvh.prototype.getAabbMax=function(nodeIndex)
	{
		if (this.m_useQuantization)
		{
			return this.unQuantize(this.m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMax);
		} 
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMaxOrg;
		
	};

	
	btQuantizedBvh.prototype.setInternalNodeEscapeIndex=function(nodeIndex, escapeIndex)
	{
		if (this.m_useQuantization)
		{
			this.m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = -escapeIndex;
		} 
		else
		{
			this.m_contiguousNodes[nodeIndex].m_escapeIndex = escapeIndex;
		}

	};

	btQuantizedBvh.prototype.mergeInternalNodeAabb=function(nodeIndex,newAabbMin,newAabbMax) 
	{
		if (this.m_useQuantization)
		{
			var quantizedAabbMin=vec3.create();
			var quantizedAabbMax=vec3.create();
			this.quantize(quantizedAabbMin,newAabbMin,0);
			this.quantize(quantizedAabbMax,newAabbMax,1);
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
btQuantizedBvh.prototype.quantizeWithClamp=function(out,point,isMax) {

    vec3.set(point,out);
    for (var i=0;i<3;++i) {
        if (!(point[i]>=this.m_bvhAabbMin[i]))
            point[i]=this.m_bvhAabbMin[i];
        if (!(point[i]<=this.m_bvhAabbMax[i]))
            point[i]=this.m_bvhAabbMax[i];
    }
}

btQuantizedBvh.prototype.unQuantize=function(vecIn)
	{
        return vecIn;
	};

btQuantizedBvh.MAX_SUBTREE_SIZE_IN_BYTES=65536*256;
	///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
	btQuantizedBvh.prototype.setTraversalMode=function(traversalMode)
	{
		this.m_traversalMode = traversalMode;
	};


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

btQuantizedBvh.prototype.buildInternal=function()
{
	///assumes that caller filled in the m_quantizedLeafNodes
	this.m_useQuantization = true;
	var numLeafNodes = 0;
	
	if (this.m_useQuantization)
	{
		//now we have an array of leafnodes in m_leafNodes
		numLeafNodes = this.m_quantizedLeafNodes.length;

		var m_quantizedContiguousNodes=new Array(2*numLeafNodes);

	}

	var m_curNodeIndex = 0;

	this.buildTree(0,numLeafNodes);

	///if the entire tree is small then subtree size, we need to create a header info for the tree
	if(this.m_useQuantization && !this.m_SubtreeHeaders.length)
	{
        var subtree;
        this.m_SubtreeHeaders.push(subtree=new btBvhSubtreeInfo());
		subtree.setAabbFromQuantizeNode(this.m_quantizedContiguousNodes[0]);
		subtree.m_rootNodeIndex = 0;
		subtree.m_subtreeSize = this.m_quantizedContiguousNodes[0].isLeafNode() ? 1 : this.m_quantizedContiguousNodes[0].getEscapeIndex();
	}

	//PCK: update the copy of the size
	this.m_subtreeHeaderCount = m_SubtreeHeaders.length;

	//PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
	this.m_quantizedLeafNodes=[];
	this.m_leafNodes=[];
};



btQuantizedBvh.prototype.setQuantizationValues=function(bvhAabbMin,bvhAabbMax,quantizationMargin)
{
    if (quantizationMargin===undefined) {
        quantizationMargin=0.0;//FIXME was 1.0 in bullet
    }
	//enlarge the AABB to avoid division by zero when initializing the quantization values
	var clampValue=vec3.create([quantizationMargin,quantizationMargin,quantizationMargin]);
    vec3.subtract(bvhAabbMin, clampValue,this.m_bvhAabbMin);
	vec3.add(bvhAabbMax, clampValue,this.m_bvhAabbMax);
    var aabbSize=vec3.create();
	vec3.subtract(this.m_bvhAabbMax, this.m_bvhAabbMin,aabbSize);
	this.m_bvhQuantization = vec3.create([65533.0/aabbSize[0],65533.0/aabbSize[1],65533.0 / aabbSize[2]]);
	this.m_useQuantization = true;
};




btQuantizedBvh.prototype.buildTree=function(startIndex, endIndex)
{


	var splitAxis;
    var splitIndex;
    var i;
	var numIndices =endIndex-startIndex;
	var curIndex = this.m_curNodeIndex;

	if(numIndices<=0&&console){
        console.log("Number of indices is less or equal to zero "+endIndex+"-"+startIndex+"="+numIndices);
    }

	if (numIndices==1)
	{
		
		this.assignInternalNodeFromLeafNode(this.m_curNodeIndex,startIndex);

		this.m_curNodeIndex++;
		return;	
	}
	//calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.
	
	splitAxis = this.calcSplittingAxis(startIndex,endIndex);

	splitIndex = this.sortAndCalcSplittingIndex(startIndex,endIndex,splitAxis);

	var internalNodeIndex = this.m_curNodeIndex;
	
	//set the min aabb to 'inf' or a max value, and set the max aabb to a -inf/minimum value.
	//the aabb will be expanded during buildTree/mergeInternalNodeAabb with actual node values
	this.setInternalNodeAabbMin(this.m_curNodeIndex,this.m_bvhAabbMax);//can't use btVector3(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY)) because of quantization
	this.setInternalNodeAabbMax(this.m_curNodeIndex,this.m_bvhAabbMin);//can't use btVector3(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY)) because of quantization
	
	
	for (i=startIndex;i<endIndex;i++)
	{
		this.mergeInternalNodeAabb(this.m_curNodeIndex,this.getAabbMin(i),this.getAabbMax(i));
	}

	this.m_curNodeIndex++;
	

	//internalNode->m_escapeIndex;
	
	var leftChildNodexIndex = this.m_curNodeIndex;

	//build left child tree
	this.buildTree(startIndex,splitIndex);

	var rightChildNodexIndex = this.m_curNodeIndex;
	//build right child tree
	this.buildTree(splitIndex,endIndex);


	var escapeIndex = this.m_curNodeIndex - curIndex;

	if (this.m_useQuantization)
	{
		//escapeIndex is the number of nodes of this subtree
		var sizeQuantizedNode =32;//sizeof(btQuantizedBvhNode);
		var treeSizeInBytes = escapeIndex * sizeQuantizedNode;
		if (treeSizeInBytes > btQuantizedBvh.MAX_SUBTREE_SIZE_IN_BYTES)
		{
			this.updateSubtreeHeaders(leftChildNodexIndex,rightChildNodexIndex);
		}
	} else
	{

	}

	this.setInternalNodeEscapeIndex(internalNodeIndex,escapeIndex);

}

btQuantizedBvh.prototype.updateSubtreeHeaders=function(leftChildNodexIndex, rightChildNodexIndex)
{

    var sizeofbtQuantizedBvhNode=32;
	var leftChildNode = m_quantizedContiguousNodes[leftChildNodexIndex];
	var leftSubTreeSize = leftChildNode.isLeafNode() ? 1 : leftChildNode.getEscapeIndex();
	var leftSubTreeSizeInBytes =  leftSubTreeSize * sizeofbtQuantizedBvhNode;
	
	btQuantizedBvhNode& rightChildNode = m_quantizedContiguousNodes[rightChildNodexIndex];
	var rightSubTreeSize = rightChildNode.isLeafNode() ? 1 : rightChildNode.getEscapeIndex();
	var rightSubTreeSizeInBytes =  rightSubTreeSize *  sizeof(btQuantizedBvhNode);

	if(leftSubTreeSizeInBytes <= btQuantizedBvh.MAX_SUBTREE_SIZE_IN_BYTES)
	{
		var subtree;
        this.m_SubtreeHeaders.push(subtree=new btBvhSubtreeInfo());
		subtree.setAabbFromQuantizeNode(leftChildNode);
		subtree.m_rootNodeIndex = leftChildNodexIndex;
		subtree.m_subtreeSize = leftSubTreeSize;
	}

	if(rightSubTreeSizeInBytes <= btQuantizedBvh.MAX_SUBTREE_SIZE_IN_BYTES)
	{
        var subtree;
		this.m_SubtreeHeaders.push(subtree=new btBvhSubtreeInfo());
		subtree.setAabbFromQuantizeNode(rightChildNode);
		subtree.m_rootNodeIndex = rightChildNodexIndex;
		subtree.m_subtreeSize = rightSubTreeSize;
	}

	//PCK: update the copy of the size
	this.m_subtreeHeaderCount = m_SubtreeHeaders.size();
};


btQuantizedBvh.prototype.sortAndCalcSplittingIndex=function(startIndex,endIndex,splitAxis)
{
	var i;
	var splitIndex =startIndex;
	var numIndices = endIndex - startIndex;
	var splitValue=0.0;

	var means=vec3.create([0,0,0]);
    var center=vec3.create();
	for (i=startIndex;i<endIndex;i++)
	{
        vec3.add(this.getAabbMax(i),this.getAabbMin(i),center);
        vec3.scale(center,0.5);
		vec3.add(means,center);
	}
	vec3.scale(means, (1.0)/numIndices);
	
	splitValue = means[splitAxis];
	
	//sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
	for (i=startIndex;i<endIndex;i++)
	{
        vec3.add(this.getAabbMax(i),this.getAabbMin(i),center);
        vec3.scale(center,0.5);
		if (center[splitAxis] > splitValue)
		{
			//swap
			this.swapLeafNodes(i,splitIndex);
			splitIndex++;
		}
	}

	//if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
	//otherwise the tree-building might fail due to stack-overflows in certain cases.
	//unbalanced1 is unsafe: it can cause stack overflows
	//bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

	//unbalanced2 should work too: always use center (perfect balanced trees)	
	//bool unbalanced2 = true;

	//this should be safe too:
	var rangeBalancedIndices = numIndices/3;
	var unbalanced = ((splitIndex<=(startIndex+rangeBalancedIndices)) || (splitIndex >=(endIndex-1-rangeBalancedIndices)));
	
	if (unbalanced)
	{
		splitIndex = startIndex+ (numIndices>>1);
	}

	var unbal = (splitIndex==startIndex) || (splitIndex == (endIndex));
    if ((!unbal)&&console) {
        console.log("Tree is unbalanced");
    }

	return splitIndex;
};


btQuantizedBvh.prototype.calcSplittingAxis=function(startIndex,endIndex)
{
	var i;

	var means=vec3.create([0,0,0]);
	var variance=vec3.create([0,0,0]);
	var numIndices = endIndex-startIndex;
    var center=vec3.create();
	for (i=startIndex;i<endIndex;i++)
	{
        vec3.add(this.getAabbMax(i),this.getAabbMin(i),center);
        vec3.scale(center,0.5);
		vec3.add(means,center);
	}
	vec3.scale(means, 1.0/numIndices);
		
	for (i=startIndex;i<endIndex;i++)
	{
        vec3.add(this.getAabbMax(i),this.getAabbMin(i),center);
        vec3.scale(center,0.5);
        vec3.subtract(center,means);
		center[0]*=center[0];
        center[1]*=center[1];
        center[2]*=center[2];
		vec3.add(variance, center);
	}
	vec3.scale(variance, 1./(numIndices-1));
	var maxaxis=0;
    if (variance[1]>variance[maxaxis])
        maxaxis=1;
    if (variance[2]>variance[maxaxis]) {
        maxaxis=2;
    }
	return maxaxis;
}



btQuantizedBvh.prototype.reportAabbOverlappingNodex=function(nodeCallback,aabbMin,aabbMax) 
{
	//either choose recursive traversal (walkTree) or stackless (walkStacklessTree)

	if (this.m_useQuantization)
	{
		///quantize query AABB
		var quantizedQueryAabbMin=vec3.create();
		var quantizedQueryAabbMax=vec3.create();
		this.quantizeWithClamp(quantizedQueryAabbMin,aabbMin,0);
		this.quantizeWithClamp(quantizedQueryAabbMax,aabbMax,1);

		switch (m_traversalMode)
		{
		case btQuantizedBvh.TRAVERSAL_STACKLESS:
				this.walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,0,this.m_curNodeIndex);
			break;
		case btQuantizedBvh.TRAVERSAL_STACKLESS_CACHE_FRIENDLY:
				this.walkStacklessQuantizedTreeCacheFriendly(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
			break;
		case btQuantizedBvh.TRAVERSAL_RECURSIVE:
			{
				this.walkRecursiveQuantizedTreeAgainstQueryAabb(0,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
			}
			break;
		default:
			//unsupported
			break;
		}
	} else
	{
		this.walkStacklessTree(nodeCallback,aabbMin,aabbMax);
	}
};


var maxIterations = 0;


/**
 * @param {function} nodeCallback  takes in subpartid (0) and triangleIndex
 * @param {Float32Array} aabbMin
 * @param {Float32Array} aabbMax
 */
btQuantizedBvh.prototype.walkStacklessTree=function(nodeCallback,aabbMin,aabbMax)
{
	if (this.m_useQuantization&&console){
        console.log("walkStacklessTree when quantization is on");
    }

	var rootNode = this.m_contiguousNodes[0];
	var escapeIndex, curIndex = 0;
	var walkIterations = 0;
	var isLeafNode;
	//PCK: unsigned instead of bool
	var aabbOverlap;

	while (curIndex < m_curNodeIndex)
	{
		//catch bugs in tree data
		btAssert (walkIterations < m_curNodeIndex);

		walkIterations++;
		aabbOverlap = TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode.m_aabbMinOrg,rootNode.m_aabbMaxOrg);
		isLeafNode = rootNode.m_escapeIndex == -1;
		
		//PCK: unsigned instead of bool
		if (isLeafNode && (aabbOverlap != 0))
		{
			nodeCallback(rootNode.m_subPart,rootNode.m_triangleIndex);
		} 
		
		//PCK: unsigned instead of bool
		if ((aabbOverlap != 0) || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode.m_escapeIndex;
			curIndex += escapeIndex;
			rootNode = this.m_contiguousNodes[curIndex];
		}
	}
	if (maxIterations < walkIterations)
		maxIterations = walkIterations;
};

/*
///this was the original recursive traversal, before we optimized towards stackless traversal
void	btQuantizedBvh::walkTree(btOptimizedBvhNode* rootNode,btNodeOverlapCallback* nodeCallback,const btVector3& aabbMin,const btVector3& aabbMax) const
{
	bool isLeafNode, aabbOverlap = TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode->m_aabbMin,rootNode->m_aabbMax);
	if (aabbOverlap)
	{
		isLeafNode = (!rootNode->m_leftChild && !rootNode->m_rightChild);
		if (isLeafNode)
		{
			nodeCallback->processNode(rootNode);
		} else
		{
			walkTree(rootNode->m_leftChild,nodeCallback,aabbMin,aabbMax);
			walkTree(rootNode->m_rightChild,nodeCallback,aabbMin,aabbMax);
		}
	}

}
*/
/**
 * @param {btQuantizedBvhNode} currentNode 
 * @param {function} nodeCallback takes in submeshid (0) and triangle index
 * @param {Float32Array} quantizedQueryAabbMin
 * @param {Float32Array} quantizedQueryAabbMax
 */
btQuantizedBvh.prototype.walkRecursiveQuantizedTreeAgainstQueryAabb=function(currentNodeIndex,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax)
{
	if ((!this.m_useQuantization)&&console){
        console.log("Walk recursive quantized tree called with non quantized bvh");
    }
	var currentNode=this.m_quantizedContiguousNodes[currentNodeIndex];
	var isLeafNode;
	//PCK: unsigned instead of bool
	var aabbOverlap;

	//PCK: unsigned instead of bool
	aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,currentNode.m_quantizedAabbMin,currentNode.m_quantizedAabbMax);
	isLeafNode = currentNode.isLeafNode();
		
	//PCK: unsigned instead of bool
	if (aabbOverlap != 0)
	{
		if (isLeafNode)
		{
			nodeCallback.processNode(currentNode.getPartId(),currentNode.getTriangleIndex());
		} else
		{
			//process left and right children
			var leftChildNodeIndex = currentNodeIndex+1;
			walkRecursiveQuantizedTreeAgainstQueryAabb(leftChildNodeIndex,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
            var leftChildNode=this.m_quantizedContigousNodes[leftChildNodeIndex];
			var rightChildNodeIndex = leftChildNode.isLeafNode() ? leftChildNodeIndex+1:leftChildNodeIndex+leftChildNode.getEscapeIndex();
			this.walkRecursiveQuantizedTreeAgainstQueryAabb(rightChildNodeIndex,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
		}
	}		
};


/**
 * @param {function} nodeCallback function taking both subtreeIndex(0) and triangleIndex 
 * @param {Float32Array} raySource vec3 indicating where the ray stars
 * @param {Float32Array} rayTarget vec3 showing the last location of the ray
 * @param {Float32Array} aabbMin the minimum box cast extents
 * @param {Float32Array} aabbMin the maximum box cast extents
 * @param {number} startNodeIndex Unused (NOT SURE WHY) FIXME?
 * @param {number} endNodeIndex
 */
btQuantizedBvh.prototype.walkStacklessTreeAgainstRay=function(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, startNodeIndex, endNodeIndex)
{
    if (this.m_useQuantization&&console) {
        console.log("calling walkStacklessTree with using quantization");
    }

	var rootNode = this.m_contiguousNodes[0];
	var escapeIndex, curIndex = 0;
	var walkIterations = 0;
	var isLeafNode;
	//PCK: unsigned instead of bool
	var aabbOverlap=0;
	var rayBoxOverlap=0;
	var lambda_max = 1.0;
	
		/* Quick pruning by quantized box */
	var rayAabbMin = vec3.create([raySource[0]<rayTarget[0]?raySource[0]:rayTarget[0],
                                  raySource[0]<rayTarget[1]?raySource[1]:rayTarget[1],
                                  raySource[0]<rayTarget[2]?raySource[2]:rayTarget[2]]);
	var rayAabbMax = vec3.create([raySource[0]>rayTarget[0]?raySource[0]:rayTarget[0],
                                  raySource[0]>rayTarget[1]?raySource[1]:rayTarget[1],
                                  raySource[0]>rayTarget[2]?raySource[2]:rayTarget[2]]);

	/* Add box cast extents to bounding box */
	vec3.add(rayAabbMin, aabbMin);
	vec3.add(rayAabbMax, aabbMax);
    var RAYAABB2=false;
    var rayDir,rayDirectionInverse,sign;
    if (RAYAABB2) {
   
	    rayDir = vec3.create();
        vec3.subtract(rayTarget,raySource,rayDir);
	    vec3.normalize(rayDir);
	    lambda_max = rayDir.dot(rayTarget-raySource);
	    ///what about division by zero? --> just set rayDirection[i] to 1.0
	    rayDirectionInverse=vec3.create();
	    rayDirectionInverse[0] = rayDir[0] == 0.0 ? BT_LARGE_FLOAT : 1.0 / rayDir[0];
	    rayDirectionInverse[1] = rayDir[1] == 0.0 ? BT_LARGE_FLOAT : 1.0 / rayDir[1];
	    rayDirectionInverse[2] = rayDir[2] == 0.0 ? BT_LARGE_FLOAT : 1.0 / rayDir[2];
	    sign = [ rayDirectionInverse[0] < 0.0, rayDirectionInverse[1] < 0.0, rayDirectionInverse[2] < 0.0];
    }

	var bounds=[vec3.create(),vec3.create()];
    var paramArray=[1.0];
	while (curIndex < m_curNodeIndex)
	{
		var param = 1.0;
        paramArray[0]=param;
		//catch bugs in tree data
		if (walkIterations >= this.m_curNodeIndex&&console){
            console.log ("Walk iterations of "+walkIterations+" should not be >= "+this.m_curNodeIndex);
        }

		walkIterations++;

		vec3.subtract(rootNode.m_aabbMinOrg,aabbMax,bounds[0]);
        vec3.subtract(rootNode.m_aabbMaxOrg,aabbMin,bounds[1]);

		aabbOverlap = TestAabbAgainstAabb2(rayAabbMin,rayAabbMax,rootNode.m_aabbMinOrg,rootNode.m_aabbMaxOrg);
		//perhaps profile if it is worth doing the aabbOverlap test first

        if(RAYAABB2) {
            
			///careful with this check: need to check division by zero (above) and fix the unQuantize method
			///thanks Joerg/hiker for the reproduction case!
			///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
		    rayBoxOverlap = aabbOverlap ? btRayAabb2 (raySource, rayDirectionInverse, sign, bounds, paramArray, 0.0, lambda_max) : false;
            
        }else {
            
		    var normal=vec3.create();
		    rayBoxOverlap = btRayAabb(raySource, rayTarget,bounds[0],bounds[1],paramArray, normal);
        }

		isLeafNode = rootNode.m_escapeIndex == -1;
		
		//PCK: unsigned instead of bool
		if (isLeafNode && (rayBoxOverlap != 0))
		{
			nodeCallback(rootNode.m_subPart,rootNode.m_triangleIndex);
		} 
		
		//PCK: unsigned instead of bool
		if ((rayBoxOverlap != 0) || isLeafNode)
		{
			curIndex++;
            rootNode=this.m_contiguousNodes[curIndex];
		} else
		{
			escapeIndex = rootNode.m_escapeIndex;
			curIndex += escapeIndex;
			rootNode = this.m_contiguousNodex[curIndex];
		}
	}
	if (maxIterations < walkIterations)
		maxIterations = walkIterations;

};


/**
 * @param {function} nodeCallback function taking both subtreeIndex(0) and triangleIndex 
 * @param {Float32Array} raySource vec3 indicating where the ray stars
 * @param {Float32Array} rayTarget vec3 showing the last location of the ray
 * @param {Float32Array} aabbMin the minimum box cast extents
 * @param {Float32Array} aabbMin the maximum box cast extents
 * @param {number} startNodeIndex Root node
 * @param {number} endNodeIndex beyond last node of subtree
 */

btQuantizedBvh.prototype.walkStacklessQuantizedTreeAgainstRay=function(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, startNodeIndex,endNodeIndex)
{
	if ((!this.m_useQuantization)&&console){
        console.log("Walk stackless quantized tree without using quantization");
    }
	
	var curIndex = startNodeIndex;
	var walkIterations = 0;
	var subTreeSize = endNodeIndex - startNodeIndex;

	var rootNode = this.m_quantizedContiguousNodes[startNodeIndex];
	var escapeIndex;
	
	var isLeafNode;
	//PCK: unsigned instead of bool
	var boxBoxOverlap = 0;
	var rayBoxOverlap = 0;

	var lambda_max = 1.0;
    var RAYAABB2=false;
    var rayDirection=vec3.create();
    var sign;
    if(RAYAABB2) {
        var rayDelta=vec3.create();
        vec3.subtract(rayTarget,raySource,rayDelta);
	    vec3.scale(rayDelta,vec3.length(rayDelta),rayDirection);
	    lambda_max = vec3.dot(rayDirection,rayDelta);
	///what about division by zero? --> just set rayDirection[i] to 1.0
	    rayDirection[0] = rayDirection[0] == 0.0 ? BT_LARGE_FLOAT : 1.0 / rayDirection[0];
	    rayDirection[1] = rayDirection[1] == 0.0 ? BT_LARGE_FLOAT : 1.0 / rayDirection[1];
	    rayDirection[2] = rayDirection[2] == 0.0 ? BT_LARGE_FLOAT : 1.0 / rayDirection[2];
	    sign= [rayDirection[0] < 0.0, rayDirection[1] < 0.0, rayDirection[2] < 0.0];
    }

	/* Quick pruning by quantized box */
	var rayAabbMin = vec3.create(raySource);
	var rayAabbMax = vec3.create(raySource);
    for (var i=0;i<3;++i){
	    if (rayAabbMin[i]>rayTarget[i])
            rayAabbMin[i]=rayTarget[i];
	    if (rayAabbMax[i]<rayTarget[i])
            rayAabbMax[i]=rayTarget[i];
    }

	/* Add box cast extents to bounding box */
	vec3.add(rayAabbMin, aabbMin);
	vec3.add(rayAabbMax, aabbMax);

	var quantizedQueryAabbMin=vec3.create();
    var quantizedQueryAabbMax=vec3.create();
	this.quantizeWithClamp(quantizedQueryAabbMin,rayAabbMin,0);
	this.quantizeWithClamp(quantizedQueryAabbMax,rayAabbMax,1);
    var paramArray=[1.0];
	while (curIndex < endNodeIndex)
	{


		//catch bugs in tree data
		if (walkIterations >= subTreeSize&&console){
            console.log("Bug in tree data, passing subTreeSize");
        }

		walkIterations++;
		//PCK: unsigned instead of bool
		// only interested if this is closer than any previous hit
		var param = 1.0;
        paramArray[0]=param;
		rayBoxOverlap = 0;
		boxBoxOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode.m_quantizedAabbMin,rootNode.m_quantizedAabbMax);
		isLeafNode = rootNode.isLeafNode();
		if (boxBoxOverlap)
		{
			var bounds=[vec3.create(),vec3.create()];
			vec3.subtract(this.unQuantize(rootNode.m_quantizedAabbMin),aabbMax,bounds[0]);
			vec3.subtract(this.unQuantize(rootNode.m_quantizedAabbMax),aabbMin,bounds[1]);
			var normal=vec3.create();
            if (RAYAABB2) {    
			    ///careful with this check: need to check division by zero (above) and fix the unQuantize method
			    ///thanks Joerg/hiker for the reproduction case!
			    ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
                
			    //BT_PROFILE("btRayAabb2");
			    rayBoxOverlap = btRayAabb2 (raySource, rayDirection, sign, bounds, paramArray, 0.0, lambda_max);
			    
            }else {
                
			    rayBoxOverlap = btRayAabb(raySource, rayTarget, bounds[0], bounds[1], paramArray, normal);//FIXME DRH this was set to "true" at all times... that does not seem rational
            }
		}
		
		if (isLeafNode && rayBoxOverlap)
		{
			nodeCallback(rootNode.getPartId(),rootNode.getTriangleIndex());
		}
		
		//PCK: unsigned instead of bool
		if ((rayBoxOverlap != 0) || isLeafNode)
		{
			curIndex++;
			rootNode=this.m_quantizedContiguousNodes[curIndex];
		} else
		{
			escapeIndex = rootNode.getEscapeIndex();
			curIndex += escapeIndex;
			rootNode=this.m_quantizedContiguousNodes[curIndex];
		}
	}
	if (maxIterations < walkIterations)
		maxIterations = walkIterations;

};

btQuantizedBvh.prototype.walkStacklessQuantizedTree=function(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax, startNodeIndex, endNodeIndex)
{
	if ((!this.m_useQuantization)&&console) {
        console.log("Walk stackless quantized tree called on nonquantized tree");
    }
	
	var curIndex = startNodeIndex;
	var walkIterations = 0;
	var subTreeSize = endNodeIndex - startNodeIndex;

	var rootNode = this.m_quantizedContiguousNodes[startNodeIndex];
	var escapeIndex;
	
	var isLeafNode;
	//PCK: unsigned instead of bool
	var aabbOverlap;

	while (curIndex < endNodeIndex)
	{


		//catch bugs in tree data
		if (walkIterations >= subTreeSize&&console){
            console.log("Walk iterations failed in subtree dataset");
        }

		walkIterations++;
		//PCK: unsigned instead of bool
		aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode.m_quantizedAabbMin,rootNode.m_quantizedAabbMax);
		isLeafNode = rootNode.isLeafNode();
		
		if (isLeafNode && aabbOverlap)
		{
			nodeCallback(rootNode.getPartId(),rootNode.getTriangleIndex());
		} 
		
		//PCK: unsigned instead of bool
		if ((aabbOverlap != 0) || isLeafNode)
		{
			curIndex++;
			rootNode=this.m_quantizedContiguousNodes[curIndex];

		} else
		{
			escapeIndex = rootNode.getEscapeIndex();
			curIndex += escapeIndex;
            rootNode=this.m_quantizedContiguousNodes[startNodeIndex];
		}
	}
	if (maxIterations < walkIterations)
		maxIterations = walkIterations;

};

//This traversal can be called from Playstation 3 SPU
btQuantizedBvh.prototype.walkStacklessQuantizedTreeCacheFriendly=function(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax)
{
	if((!this.m_useQuantization)&&console) {
        console.log("Walking stackless quantized tree without quantization");
    }

	var i;

    var subtreeLen=this.m_SubtreeHeaders.length;
	for (i=0;i<subtreeLen;i++)
	{
		var subtree = m_SubtreeHeaders[i];

		//PCK: unsigned instead of bool
		var overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
		if (overlap != 0)
		{
			walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,
				subtree.m_rootNodeIndex,
				subtree.m_rootNodeIndex+subtree.m_subtreeSize);
		}
	}
};


btQuantizedBvh.prototype.reportRayOverlappingNodex=function (nodeCallback, raySource, rayTarget)
{
	this.reportBoxCastOverlappingNodex(nodeCallback,raySource,rayTarget,vec3.create([0,0,0]),vec3.create([0,0,0]));
};


btQuantizedBvh.prototype.reportBoxCastOverlappingNodex=function(nodeCallback, raySource, rayTarget, aabbMin,aabbMax)
{
	//always use stackless

	if (this.m_useQuantization)
	{
		this.walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, this.m_curNodeIndex);
	}
	else
	{
		this.walkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, this.m_curNodeIndex);
	}
	/*
	{
		//recursive traversal
		btVector3 qaabbMin = raySource;
		btVector3 qaabbMax = raySource;
		qaabbMin.setMin(rayTarget);
		qaabbMax.setMax(rayTarget);
		qaabbMin += aabbMin;
		qaabbMax += aabbMax;
		reportAabbOverlappingNodex(nodeCallback,qaabbMin,qaabbMax);
	}
	*/

};


btQuantizedBvh.prototype.swapLeafNodes=function(i, splitIndex)
{
	if (this.m_useQuantization)
	{
			var tmp = this.m_quantizedLeafNodes[i];
			this.m_quantizedLeafNodes[i] = this.m_quantizedLeafNodes[splitIndex];
			this.m_quantizedLeafNodes[splitIndex] = tmp;
	} else
	{
			var tmp = this.m_leafNodes[i];
			this.m_leafNodes[i] = this.m_leafNodes[splitIndex];
			this.m_leafNodes[splitIndex] = tmp;
	}
};
///FIXME not sure pointer assigment is desirable/sufficient
btQuantizedBvh.prototype.assignInternalNodeFromLeafNode=function(internalNode, leafNodeIndex)
{
	if (this.m_useQuantization)
	{
		this.m_quantizedContiguousNodes[internalNode] = this.m_quantizedLeafNodes[leafNodeIndex];
	} else
	{
		this.m_contiguousNodes[internalNode] = this.m_leafNodes[leafNodeIndex];
	}
};

