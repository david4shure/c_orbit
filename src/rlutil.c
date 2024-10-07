#include <raylib.h>
#include <rlgl.h>

// Draw a grid centered at (0, 0, 0)
void DrawGridOfColor(int slices, float spacing,Color color)
{
    int halfSlices = slices/2;

    rlBegin(RL_LINES);
        for (int i = -halfSlices; i <= halfSlices; i++)
        {
            if (i == 0)
            {
                rlColor3f(color.r*255,color.g*255,color.b*255);
                rlColor3f(color.r*255,color.g*255,color.b*255);
                rlColor3f(color.r*255,color.g*255,color.b*255);
                rlColor3f(color.r*255,color.g*255,color.b*255);
            }
            else
            {
                rlColor3f(color.r*255,color.g*255,color.b*255);
                rlColor3f(color.r*255,color.g*255,color.b*255);
                rlColor3f(color.r*255,color.g*255,color.b*255);
                rlColor3f(color.r*255,color.g*255,color.b*255);
            }

            rlVertex3f((float)i*spacing, 0.0f, (float)-halfSlices*spacing);
            rlVertex3f((float)i*spacing, 0.0f, (float)halfSlices*spacing);

            rlVertex3f((float)-halfSlices*spacing, 0.0f, (float)i*spacing);
            rlVertex3f((float)halfSlices*spacing, 0.0f, (float)i*spacing);
        }
    rlEnd();
}

