

/**
 * 
 * 
 * The mesh looks like
 * xy view
 *  []/
 *  \
 * z view
 * 
 * =
 * [][]
 * 
 */

function testbvh() {
    var vertices=[-1,0,-1,//0
                  -1,0,0,
                  0,-1,0,//2
                  0,-1,-1,
                  0,0,-1,//4
                  0,0,0,
                  1,1,0,//6
                  1,1,-1,
                  -1,1,0,//8
                  0,1,0,
                  0,0,.5,//10
                  -1,0,.5,
                  -1,1,1,//12
                  0,1,1,
                  0,0,1,//14
                  -1,0,1];
    var vertexStride=3;
    var indices=[0,1,2,
                 0,2,3,
                4,5,6,
                4,6,7,
                8,9,10,
                8,10,11,
                12,13,14,
                 12,14,15
                ];
    var indexStride=3;
    var scale=[2.0,1.0,0.5];
    var curRay=[];
    var meshInterface = new btStridingMeshInterface(scale,vertices,vertexStride,indices,indexStride);
    var triangleMeshShape=new btBvhTriangleMeshShape(meshInterface,true,[-2.0,-1.0,-0.5],[2.0,1.0,0.5],true);
    function cb0(ax,ay,az,bx,by,bz,cx,cy,cz,partIndex,triangleIndex){
        console.log("Hit triangle "+triangleIndex+" on part "+partIndex +" coords: "+[[ax,ay,az],[bx,by,bz],[cx,cy,cz]]+" with ray "+curRay[0][0]+","+curRay[0][1]+","+curRay[0][2]+" to "+curRay[1][0]+","+curRay[1][1]+","+curRay[1][2]);
    }
    function cb1(ax,ay,az,bx,by,bz,cx,cy,cz,partIndex,triangleIndex){
        console.log("Should have missed triangle "+triangleIndex+" on part "+partIndex +" coords: "+[[ax,ay,az],[bx,by,bz],[cx,cy,cz]]+" with ray "+curRay[0][0]+","+curRay[0][1]+","+curRay[0][2]+" to "+curRay[1][0]+","+curRay[1][1]+","+curRay[1][2]);
    }
    curRay=[[-.5,-10,-.25],[-.5,0,-.125]];
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[-1.0,-10,-.25],[-1.0,0,-.125]]
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[-1.5,-10,-.25],[-1.5,0,-.125]];
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[-.5,-10,-.75],[-.5,0,-.75]];
    triangleMeshShape.performRaycast(cb1,curRay[0],curRay[1]);
    curRay=[[1,0,0],[0,1,-.25]];
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[1,0,-1.25],[0,1.25,-.33]];
    triangleMeshShape.performRaycast(cb1,curRay[0],curRay[1]);

    curRay=[[-1,-1,0.2],[-1,1,0.2]];
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[-1,-1,0.25],[-1,1,0.55]];
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[-1.5,-1,0.2],[-1.5,1,0.2]];
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[-1.5,-1,0.25],[-1.5,1,0.55]];
    triangleMeshShape.performRaycast(cb0,curRay[0],curRay[1]);
    curRay=[[-2.5,-1,0.2],[-2.5,1,0.2]];
    triangleMeshShape.performRaycast(cb1,curRay[0],curRay[1]);
    curRay=[[-2.5,-1,0.25],[-2.5,1,0.55]];
    triangleMeshShape.performRaycast(cb1,curRay[0],curRay[1]);
}
