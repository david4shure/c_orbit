#include <raylib.h>
#include <rlgl.h>

// Draw a grid centered at (0, 0, 0)
void DrawGridOfColor(int slices, float spacing,Color color)
{
    int halfSlices = slices/2;

    rlBegin(RL_LINES);
        for (int i = -halfSlices; i <= halfSlices; i++)
        {
            rlColor3f(color.r*color.a,color.g*color.a,color.b*color.a);
            rlColor3f(color.r*color.a,color.g*color.a,color.b*color.a);
            rlColor3f(color.r*color.a,color.g*color.a,color.b*color.a);
            rlColor3f(color.r*color.a,color.g*color.a,color.b*color.a);

            rlVertex3f((float)i*spacing, 0.0f, (float)-halfSlices*spacing);
            rlVertex3f((float)i*spacing, 0.0f, (float)halfSlices*spacing);

            rlVertex3f((float)-halfSlices*spacing, 0.0f, (float)i*spacing);
            rlVertex3f((float)halfSlices*spacing, 0.0f, (float)i*spacing);
        }
    rlEnd();
}

