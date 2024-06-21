using OpenTK.Mathematics;
using System.Drawing;

namespace Graphic
{
    public interface IDrawingContext
    {
        void ClearScreen(Color color);
        void DrawAxialRectangle(RectangleF rec, Color color);
        void DrawRotatedRectangle(Vector2 center, float width, float height, float angle, Color color);
        void DrawLine(Vector2 p1, Vector2 p2, float lineWidth, Color color);
        void SwapBuffer();
    }
}
