///for btTriangleInfo m_flags



function btTriangleInfo () {
    this.m_edgeV0V1Angle = SIMD_2_PI;
    this.m_edgeV1V2Angle = SIMD_2_PI;
    this.m_edgeV2V0Angle = SIMD_2_PI;
    this.m_flags=0;
}


btTriangleInfo.TRI_INFO_V0V1_CONVEX=1;
btTriangleInfo.TRI_INFO_V1V2_CONVEX=2;
btTriangleInfo.TRI_INFO_V2V0_CONVEX=4;

btTriangleInfo.TRI_INFO_V0V1_SWAP_NORMALB=8;
btTriangleInfo.TRI_INFO_V1V2_SWAP_NORMALB=16;
btTriangleInfo.TRI_INFO_V2V0_SWAP_NORMALB=32;


function btTriangleInfoMap(){
    this.m_convexEpsilon =0.0;
    this.m_planarEpsilon = 0.0001;
    this.m_equalVertexthreshold=0.0001*0.0001;
    this.m_edgeDistanceThreshold=0.1;
    this.m_zeroAreaThreshold = .0001*.0001;
    /**
     * @type{Object} map from btHashInt to btTriangleInfo
     */
    this.btInternalTriangleInfoMap={};
}

