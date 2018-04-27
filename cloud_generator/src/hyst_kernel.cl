// Hysteresis Threshold Kernel
// data: image input data with each pixel taking up 1 byte (8Bit 1Channel)
// out: image output data (8B1C)
__kernel void hyst_kernel(__global uchar *data,
                           __global uchar *out,
                                    int rows,
                                    int cols)
{
    // Establish our high and low thresholds as floats
    float lowThresh = 30;
    float highThresh = 70;

    // These variables are offset by one to avoid seg. fault errors
    // As such, this kernel ignores the outside ring of pixels
    int row = get_global_id(0);
    int col = get_global_id(1);
    int pos = row * cols + col;

    const uchar EDGE = 255;

    uchar magnitude = data[pos];
    
    if (magnitude >= highThresh)
        out[pos] = EDGE;
    else if (magnitude <= lowThresh)
        out[pos] = 0;
    else
    {
        float med = (highThresh + lowThresh)/2;
        
        if (magnitude >= med)
            out[pos] = EDGE;
        else
            out[pos] = 0;
    }
}
