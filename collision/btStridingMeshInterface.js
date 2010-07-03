
/**
 * @param {Float32Array} vertexArray the array of floats that will make up the vector3 positions
 * @param {number} vertexStride how many in the index array to skip over for each prim (if omitted, 3)
 * @param {number} indexStride how many in the vertex array to skip over for each prim (if omitted, 3)
 * @param {Float32Array} scale 3 vector of floats indicating scale in x y and z if missing, assumed 1
 * @param {UInt32Array} indexArray (optional) the indices that make up the mesh
 */
function btStridingMeshInterface(scale,vertexArray,vertexStride,indexStride,indexArray) {
    /**
     *@type {Float32Array}
     */
    this.m_vertexArray=vertexArray;
    if (scale===undefined) {
        scale=[1,1,1];
    }else if (typeof(scale)==typeof(1.0)){
        scale=[scale,scale,scale];
    }
    /**
     *@type {Float32Array}
     */    
    this.m_scaling=scale;
    if (!indexArray) {
        var len=vertexArray.length;
        indexArray=new Array(len);
        for (var i=0;i<len;++i) {
            indexArray[i]=i;
        }
    }
    /**
     * @type {UInt32Array}
     */
    this.m_indexArray=indexArray;
    /**
     * @type {number}
     */
    this.m_vertexStride=vertexStride?vertexStride:3;
    /**
     * @type {number}
     */
    this.m_indexStride=indexStride?indexStride:3;
}

btStridingMeshInterface.InternalProcessAllTriangles=function(callback,aabbMin,aabbMax){
    var len=this.m_indexArray.length-stride+1;
    var stride=this.m_indexStride;
    var vertexArray=this.m_vertexArray;
    var indexArray=this.m_indexArray;
    var scale=this.m_scaling;
    var vStride=this.m_vertexStride;
    for (var i=0;i<len;i+=stride) {
       callback(vertexArray[indexArray[i]*vStride]*scale[0], 
                vertexArray[indexArray[i]*vStride+1]*scale[1], 
                vertexArray[indexArray[i]*vStride+2]*scale[2], 
                vertexArray[indexArray[i+1]*vStride]*scale[0], 
                vertexArray[indexArray[i+1]*vStride+1]*scale[1], 
                vertexArray[indexArray[i+1]*vStride+2]*scale[2], 
                vertexArray[indexArray[i+2]*vStride]*scale[0], 
                vertexArray[indexArray[i+2]*vStride+1]*scale[1], 
                vertexArray[indexArray[i+2]*vStride+2]*scale[2], 
                i);
    }
};

/**
 * @param {Float32Array} aabbMin 
 * @param {Float32Array} aabbMax 
 */
btStridingMeshInterface.calculateAabbBruteForce=function(aabbMin,aabbMax){
    aabbMin[0]=BT_LARGE_FLOAT;
    aabbMax[0]=-BT_LARGE_FLOAT;
    aabbMin[1]=BT_LARGE_FLOAT;
    aabbMax[1]=-BT_LARGE_FLOAT;
    aabbMin[2]=BT_LARGE_FLOAT;
    aabbMax[2]=-BT_LARGE_FLOAT;

    var len=this.m_indexArray.length-stride+1;
    var stride=this.m_indexStride;
    var vertexArray=this.m_vertexArray;
    var indexArray=this.m_indexArray;
    var scale=this.m_scaling;    
    var vStride=this.m_vertexStride;
    for (var i=0;i<len;i+=stride) {
        for (var j=0;j<3;++j){
            var val=vertexArray[indexArray[i]*vStride+j]*scale[j];
            if (aabbMin[j]>val)
                aabbMin[j]=val;
            if (aabbMax[j]<val)
                aabbMax[j]=val;
        }
    }
};

btStridingMeshInterface.seScaling=function(scaling) {
    this.m_scaling=scaling;
};
btStridingMeshInterface.getScaling=function() {
    return this.m_scaling;
};
