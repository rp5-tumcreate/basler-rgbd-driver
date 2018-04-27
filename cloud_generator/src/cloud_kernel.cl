typedef struct Params
{
    int height, width;
    float fx,fy,cx,cy;
    float r00, r01, r02, t0,
          r10, r11, r12, t1,
          r20, r21, r22, t2;
    float xRange, yRange, zRange;
} Params;
__kernel void cloud_kernel(__constant struct Params* cameraParams, __global ushort* depthImage, __global float* cloudOutput)
{
    int u = get_global_id(0); //width
    int v = get_global_id(1); //height
    int index = v*cameraParams->width + u; // index

    const ushort d = depthImage[index];
    if(d != 0){
        const float z = d*0.001;
        const float y = (((float)v - cameraParams->cy) * z) / (cameraParams->fy);
        const float x = (((float)u - cameraParams->cx) * z) / (cameraParams->fx);
        const float xt = x*cameraParams->r00 + y*cameraParams->r01 + z*cameraParams->r02 + cameraParams->t0;
        const float yt = x*cameraParams->r10 + y*cameraParams->r11 + z*cameraParams->r12 + cameraParams->t1;
        const float zt = x*cameraParams->r20 + y*cameraParams->r21 + z*cameraParams->r22 + cameraParams->t2;

        if(xt < -cameraParams->xRange || xt > cameraParams->xRange || yt < -cameraParams->yRange || yt > cameraParams->yRange || zt < cameraParams->zRange){
            cloudOutput[index*3 + 0] = NAN;
            cloudOutput[index*3 + 1] = NAN;
            cloudOutput[index*3 + 2] = NAN;
        }else{
            cloudOutput[index*3 + 0] = xt;
            cloudOutput[index*3 + 1] = yt;
            cloudOutput[index*3 + 2] = zt;
        }
    }else{
        cloudOutput[index*3 + 0] = NAN;
        cloudOutput[index*3 + 1] = NAN;
        cloudOutput[index*3 + 2] = NAN;
    }
}
