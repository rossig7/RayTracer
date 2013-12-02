#include "System.h"

#include "IO.h"

void saveBmp(const char *filename, int w, int h, int dpi, RGBType *data)
{
    FILE *f;
    int k = w * h;
    int s = 4 * k;
    int filesize = 54 + s;

    double factor = 39.375;
    int m = static_cast<int>(factor);

    int ppm = dpi * m;

    unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
    unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0};

    bmpFileHeader[2] = (unsigned char) (filesize);
    bmpFileHeader[3] = (unsigned char) (filesize >> 8);
    bmpFileHeader[4] = (unsigned char) (filesize >> 16);
    bmpFileHeader[5] = (unsigned char) (filesize >> 24);

    bmpInfoHeader[4] = (unsigned char) (w);
    bmpInfoHeader[5] = (unsigned char) (w >> 8);
    bmpInfoHeader[6] = (unsigned char) (w >> 16);
    bmpInfoHeader[7] = (unsigned char) (w >> 24);

    bmpInfoHeader[8] = (unsigned char) (h);
    bmpInfoHeader[9] = (unsigned char) (h >> 8);
    bmpInfoHeader[10] = (unsigned char) (h >> 16);
    bmpInfoHeader[11] = (unsigned char) (h >> 24);

    bmpInfoHeader[21] = (unsigned char) (s);
    bmpInfoHeader[22] = (unsigned char) (s >> 8);
    bmpInfoHeader[23] = (unsigned char) (s >> 16);
    bmpInfoHeader[24] = (unsigned char) (s >> 24);

    bmpInfoHeader[25] = (unsigned char) (ppm);
    bmpInfoHeader[26] = (unsigned char) (ppm >> 8);
    bmpInfoHeader[27] = (unsigned char) (ppm >> 16);
    bmpInfoHeader[28] = (unsigned char) (ppm >> 24);

    bmpInfoHeader[29] = (unsigned char) (ppm);
    bmpInfoHeader[30] = (unsigned char) (ppm >> 8);
    bmpInfoHeader[31] = (unsigned char) (ppm >> 16);
    bmpInfoHeader[32] = (unsigned char) (ppm >> 24);

    f = fopen(filename, "wb");

    fwrite(bmpFileHeader, 1, 14, f);
    fwrite(bmpInfoHeader, 1, 40, f);

    for (int i = 0; i < k; i++) {
        RGBType rgb = data[i];

        double red = data[i].r * 255;
        double green = data[i].g * 255;
        double blue = data[i].b * 255;

        unsigned char color[3] = {(unsigned char) floor(blue), (unsigned char) floor(green), (unsigned char) floor(red)};

        fwrite(color, 1, 3, f);
    }

    fclose(f);
}

void makeCube(vector<Object *>& objects,Vect corner1, Vect corner2, Color color)
{
    double c1x = corner1.getVectX();
    double c1y = corner1.getVectY();
    double c1z = corner1.getVectZ();

    double c2x = corner2.getVectX();
    double c2y = corner2.getVectY();
    double c2z = corner2.getVectZ();

    Vect A (c2x, c1y, c1z);
    Vect B (c2x, c1y, c2z);
    Vect C (c1x, c1y, c2z);

    Vect D (c2x, c2y, c1z);
    Vect E (c1x, c2y, c1z);
    Vect F (c1x, c2y, c2z);

    //left side
    objects.push_back(new Triangle(D, A, corner1, color, 2));
    objects.push_back(new Triangle(corner1, E, D, color, 2));
    //far side
    objects.push_back(new Triangle(corner2, B, A, color, 2));
    objects.push_back(new Triangle(A, D, corner2, color, 2));
    //right side
    objects.push_back(new Triangle(F, C, B, color, 2));
    objects.push_back(new Triangle(B, corner2, F, color, 2));
    //front side
    objects.push_back(new Triangle(E, corner1, C, color, 2));
    objects.push_back(new Triangle(C, F, E, color, 2));
    //top
    objects.push_back(new Triangle(D, E, F, color, 2));
    objects.push_back(new Triangle(F, corner2, D, color, 2));
    //bottom
    objects.push_back(new Triangle(corner1, A, B, color, 2));
    objects.push_back(new Triangle(B, C, corner1, color, 2));
}

void makeCornellBox(vector<Object *>& objects, Vect corner1, Vect corner2)
{
    double c1x = corner1.getVectX();
    double c1y = corner1.getVectY();
    double c1z = corner1.getVectZ();

    double c2x = corner2.getVectX();
    double c2y = corner2.getVectY();
    double c2z = corner2.getVectZ();

    Vect A (c2x, c1y, c1z);
    Vect B (c2x, c1y, c2z);
    Vect C (c1x, c1y, c2z);

    Vect D (c2x, c2y, c1z);
    Vect E (c1x, c2y, c1z);
    Vect F (c1x, c2y, c2z);


    Color red(1, 0.25, 0.25, 0);
    Color green(0.25, 1, 0.25, 0);
    Color white(1.0, 1.0, 1.0, 0);


    //left side
    objects.push_back(new Triangle(D, A, corner1, green, 20));
    objects.push_back(new Triangle(corner1, E, D, green, 20));
    //scene_objects.push_back(new Triangle(D, A, corner1, white, 20));
    //scene_objects.push_back(new Triangle(corner1, E, D, white, 20));
    //far side
    objects.push_back(new Triangle(corner2, B, A, white, 20));
    objects.push_back(new Triangle(A, D, corner2, white, 20));
    //right side
    objects.push_back(new Triangle(F, C, B, red, 20));
    objects.push_back(new Triangle(B, corner2, F, red, 20));
    //scene_objects.push_back(new Triangle(F, C, B, white, 20));
    //scene_objects.push_back(new Triangle(B, corner2, F, white, 20));
    //front side
    //scene_objects.push_back(new Triangle(E, corner1, C, white, 20));
    //scene_objects.push_back(new Triangle(C, F, E, white, 20));
    //top
    objects.push_back(new Triangle(D, E, F, white, 20));
    objects.push_back(new Triangle(F, corner2, D, white, 20));
    //bottom
    objects.push_back(new Triangle(corner1, A, B, white, 20));
    objects.push_back(new Triangle(B, C, corner1, white, 20));
}
