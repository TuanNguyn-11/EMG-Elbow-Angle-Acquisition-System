using UnityEngine;
using UnityEngine.UI;

public class EMGWaveformGraphic : Graphic
{
    [Range(16, 512)]
    public int pointCount = 50;

    [Range(1f, 10f)]
    public float lineThickness = 2f;

    public float[] samples = new float[50];

    public void SetWave(float[] newSamples)
    {
        if (newSamples == null || newSamples.Length == 0) return;
        samples = newSamples;
        pointCount = samples.Length;
        SetVerticesDirty();
    }

    protected override void OnPopulateMesh(VertexHelper vh)
    {
        vh.Clear();

        if (samples == null || samples.Length < 2)
            return;

        Rect rect = GetPixelAdjustedRect();
        float width = rect.width;
        float height = rect.height;
        float centerY = rect.y + height * 0.5f;

        for (int i = 0; i < samples.Length - 1; i++)
        {
            float x0 = rect.x + (i / (float)(samples.Length - 1)) * width;
            float x1 = rect.x + ((i + 1) / (float)(samples.Length - 1)) * width;

            float y0 = centerY + samples[i] * (height * 0.4f);
            float y1 = centerY + samples[i + 1] * (height * 0.4f);

            DrawLine(vh, new Vector2(x0, y0), new Vector2(x1, y1), lineThickness, color);
        }
    }

    private void DrawLine(VertexHelper vh, Vector2 p1, Vector2 p2, float thickness, Color col)
    {
        Vector2 dir = (p2 - p1).normalized;
        Vector2 normal = new Vector2(-dir.y, dir.x) * thickness * 0.5f;

        UIVertex v = UIVertex.simpleVert;
        v.color = col;

        int startIndex = vh.currentVertCount;

        v.position = p1 - normal;
        vh.AddVert(v);

        v.position = p1 + normal;
        vh.AddVert(v);

        v.position = p2 + normal;
        vh.AddVert(v);

        v.position = p2 - normal;
        vh.AddVert(v);

        vh.AddTriangle(startIndex, startIndex + 1, startIndex + 2);
        vh.AddTriangle(startIndex, startIndex + 2, startIndex + 3);
    }
}