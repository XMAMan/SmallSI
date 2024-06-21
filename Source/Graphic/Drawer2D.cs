using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using System.Drawing;

namespace Graphic
{
    internal class Drawer2D : IDrawingContext
    {
        private SolidQuadDrawer solidQuadDrawer;
        private Action swapBufferAction;
        public Drawer2D(SolidQuadDrawer solidQuadDrawer, Action swapBufferAction)
        {
            this.solidQuadDrawer = solidQuadDrawer;
            this.swapBufferAction = swapBufferAction;
        }

        public void ClearScreen(Color color)
        {
            GL.ClearColor(color);
            GL.Clear(ClearBufferMask.ColorBufferBit);
        }

        public void DrawAxialRectangle(RectangleF rec, Color color)
        {
            this.solidQuadDrawer.DrawAxialRectangle(rec, color);
        }

        public void DrawLine(Vector2 p1, Vector2 p2, float lineWidth, Color color)
        {
            this.solidQuadDrawer.DrawLine(p1, p2, lineWidth, color);
        }

        public void DrawRotatedRectangle(Vector2 center, float width, float height, float angle, Color color)
        {
            this.solidQuadDrawer.DrawRotatedRectangle(center, width, height, angle, color);
        }

        public void SwapBuffer()
        {
            this.swapBufferAction();
        }
    }
}
